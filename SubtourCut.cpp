#include "SubtourCut.h"

#include <vector>
#include <queue>
#include <cmath>    // std::ceil
#include <algorithm>

#include <coin/OsiRowCut.hpp>
#include <coin/OsiCuts.hpp>
#include <coin/CglTreeInfo.hpp>

using namespace std;

// =============================================================================
// Constructor
// =============================================================================
SubtourCutGenerator::SubtourCutGenerator(const Parser* parser, int numClients)
    : parserData(parser), n(numClients) {}

// =============================================================================
// clone
// CBC requiere que cada generador pueda clonarse a sí mismo.
// =============================================================================
CglCutGenerator* SubtourCutGenerator::clone() const {
    return new SubtourCutGenerator(parserData, n);
}

// =============================================================================
// findInvalidSets
//
// Detecta dos tipos de conjuntos inválidos en la solución LP actual:
//   A) Subtours: componentes conexas de clientes que no pasan por la bodega.
//   B) Violaciones de capacidad: rutas conectadas a la bodega cuya demanda
//      total supera Q.
//
// Parámetros
//   sol       : solución LP del nodo actual (variables x_ij ordenadas como
//               i*n + j, igual que CbcSolver::getVarIndex)
//   threshold : umbral para considerar una arista "activa"
//
// Retorna
//   Vector de conjuntos S. Cada S es un vector de IDs (0-indexados, bodega=0).
// =============================================================================
vector<vector<int>> SubtourCutGenerator::findInvalidSets(const double* sol,
                                                         double threshold) const {
    vector<vector<int>> invalidSets;

    // ------------------------------------------------------------------
    // Paso 1: Construir lista de adyacencia del grafo de soporte.
    // Consideramos activa la arista i->j si sol[i*n+j] > threshold.
    // ------------------------------------------------------------------
    vector<vector<int>> adj(n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i != j && sol[getVarIndex(i, j)] > threshold) {
                adj[i].push_back(j);
            }
        }
    }

    // ------------------------------------------------------------------
    // Paso 2: BFS desde la bodega (nodo 0) para marcar nodos alcanzables.
    // Un nodo es "alcanzable" si existe algún camino desde la bodega
    // siguiendo las aristas activas.
    // ------------------------------------------------------------------
    vector<bool> reachable(n, false);
    reachable[0] = true;
    queue<int> bfsQueue;
    bfsQueue.push(0);

    while (!bfsQueue.empty()) {
        int cur = bfsQueue.front();
        bfsQueue.pop();
        for (int nb : adj[cur]) {
            if (!reachable[nb]) {
                reachable[nb] = true;
                bfsQueue.push(nb);
            }
        }
    }

    // ------------------------------------------------------------------
    // Paso 3: Detectar subtours aislados.
    //
    // Todo nodo no alcanzable desde la bodega pertenece a un subtour.
    // Usamos otro BFS para agrupar estos nodos en componentes conexas.
    // Cada componente es un conjunto inválido (un subtour).
    // ------------------------------------------------------------------
    vector<bool> processed(n, false);
    processed[0] = true; // La bodega no forma parte de ningún subtour

    for (int start = 1; start < n; start++) {
        if (reachable[start] || processed[start]) continue;

        // BFS para extraer la componente completa del subtour
        vector<int> component;
        queue<int> compQueue;
        compQueue.push(start);
        processed[start] = true;

        while (!compQueue.empty()) {
            int cur = compQueue.front();
            compQueue.pop();
            component.push_back(cur);

            for (int nb : adj[cur]) {
                if (!processed[nb]) {
                    processed[nb] = true;
                    compQueue.push(nb);
                }
            }
        }

        if (!component.empty()) {
            invalidSets.push_back(component);
        }
    }

    // ------------------------------------------------------------------
    // Paso 4: Detectar violaciones de capacidad en rutas válidas.
    //
    // Una ruta es "válida estructuralmente" si parte de la bodega (nodo 0).
    // Pero puede violar la restricción de capacidad si la demanda total
    // de los clientes visitados supera Q.
    // En ese caso, el conjunto de clientes en esa ruta también es inválido
    // y necesita un corte de capacidad (Capacity Cut, que es una versión
    // reforzada del corte DFJ con k_s > 1).
    // ------------------------------------------------------------------
    int Q = parserData->getCapacity();
    vector<bool> visitedInRoute(n, false);
    visitedInRoute[0] = true;

    for (int j = 1; j < n; j++) {
        // Buscamos rutas que arrancan desde la bodega
        if (sol[getVarIndex(0, j)] <= threshold) continue;
        if (visitedInRoute[j]) continue;

        // Rastrear la ruta completa siguiendo aristas activas
        vector<int> routeNodes;
        int totalDemand = 0;
        int curr = j;

        while (curr != 0 && !visitedInRoute[curr]) {
            visitedInRoute[curr] = true;
            routeNodes.push_back(curr);
            // IDs en Parser están 1-indexados: cliente con índice LP i
            // corresponde al cliente parserData->getClients()[i+1]
            totalDemand += parserData->getClients()[curr + 1].getDemand();

            // Avanzar al siguiente nodo en la ruta
            int next = -1;
            for (int k = 0; k < n; k++) {
                if (curr != k && sol[getVarIndex(curr, k)] > threshold) {
                    next = k;
                    break;
                }
            }
            if (next == -1) break;
            curr = next;
        }

        // Si la demanda total excede la capacidad del vehículo,
        // este conjunto de nodos es inválido: agregar como capacity cut.
        if (totalDemand > Q && !routeNodes.empty()) {
            invalidSets.push_back(routeNodes);
        }
    }

    return invalidSets;
}

// =============================================================================
// generateCuts  [el método central que CBC llama en cada nodo]
//
// Estrategia de dos pasadas:
//
//   Pasada 1 (threshold = 0.5): Detección exacta de subtours enteros.
//     Si la solución del nodo ya es entera (o casi), buscamos violaciones
//     "duras" para agregar cortes precisos y fuertes.
//
//   Pasada 2 (threshold = 1e-4): Detección agresiva en soluciones fraccionales.
//     Incluso si la solución LP es fraccional, podemos detectar componentes
//     que "tienden" a ser subtours y agregar cortes preventivos.
//     Esto sube el LB más rápido, acortando el árbol de búsqueda.
//
// Para cada conjunto inválido S encontrado, se agrega el corte DFJ:
//
//   sum_{i in S} sum_{j in S, j!=i}  x_ij  <=  |S| - k_s
//
//   donde k_s = ceil( sum_{i in S} d_i / Q )
//
//   - Si S es un subtour puro (no pasa por la bodega):
//       k_s >= 1, el RHS es |S| - k_s <= |S| - 1
//   - Si S viola capacidad:
//       k_s > 1, el corte es más fuerte (RHS más chico)
//
// Los cortes se insertan con insertIfNotDuplicate para evitar redundancias.
// =============================================================================
void SubtourCutGenerator::generateCuts(const OsiSolverInterface& si,
                                       OsiCuts& cs,
                                       const CglTreeInfo /*info*/) {
    const double* sol = si.getColSolution();
    if (!sol) return;

    int Q = parserData->getCapacity();

    // Ejecutar las dos pasadas con distintos umbrales
    const double thresholds[] = {0.5, 1e-4};
    const int numPasses = 2;

    for (int pass = 0; pass < numPasses; pass++) {
        double threshold = thresholds[pass];
        vector<vector<int>> invalidSets = findInvalidSets(sol, threshold);

        if (invalidSets.empty()) continue;

        for (const auto& S : invalidSets) {
            if (S.size() < 2) continue; // No tiene sentido cortar un conjunto unitario

            // -----------------------------------------------------------------
            // Calcular k_s = ceil( demanda(S) / Q )
            // Este valor determina cuántos vehículos mínimos necesita S.
            // Si k_s == 1 tenemos un corte subtour estándar.
            // Si k_s > 1 tenemos un capacity cut más fuerte.
            // -----------------------------------------------------------------
            int demandSum = 0;
            for (int node : S) {
                // Nodo 0 es la bodega; demanda = 0, pero por seguridad lo saltamos
                if (node == 0) continue;
                demandSum += parserData->getClients()[node + 1].getDemand();
            }

            int k_s = static_cast<int>(std::ceil(static_cast<double>(demandSum) / Q));
            if (k_s < 1) k_s = 1;

            // RHS del corte: |S| - k_s
            // Para que el corte tenga sentido debe ser >= 0.
            // Si |S| <= k_s el subconjunto es infactible por sí solo (ya está
            // acotado por las restricciones de grado), así que lo saltamos.
            int rhs = static_cast<int>(S.size()) - k_s;
            if (rhs < 0) continue;

            // -----------------------------------------------------------------
            // Construir la fila del corte:
            //   sum_{i in S, j in S, i!=j}  x_ij  <=  rhs
            // -----------------------------------------------------------------
            vector<int>    indices;
            vector<double> coefficients;

            indices.reserve(S.size() * (S.size() - 1));
            coefficients.reserve(S.size() * (S.size() - 1));

            for (int u : S) {
                for (int v : S) {
                    if (u != v) {
                        indices.push_back(getVarIndex(u, v));
                        coefficients.push_back(1.0);
                    }
                }
            }

            if (indices.empty()) continue;

            // -----------------------------------------------------------------
            // Crear y registrar el corte en el objeto OsiCuts.
            // CBC revisa los duplicados internamente con insertIfNotDuplicate.
            // -----------------------------------------------------------------
            OsiRowCut cut;
            cut.setRow(static_cast<int>(indices.size()),
                       indices.data(),
                       coefficients.data());
            cut.setLb(-1.0e30);          // Sin cota inferior (desigualdad <=)
            cut.setUb(static_cast<double>(rhs));

            cs.insertIfNotDuplicate(cut);
        }
    }
}
