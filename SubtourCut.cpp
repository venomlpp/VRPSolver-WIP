#include "SubtourCut.h"

#include <vector>
#include <queue>
#include <cmath>    
#include <algorithm>

#include <coin/OsiRowCut.hpp>
#include <coin/OsiCuts.hpp>
#include <coin/CglTreeInfo.hpp>

using namespace std;

/*
 * Descripción: Constructor del generador de cortes.
 * Entrada: Puntero al parser de la instancia, número total de nodos (incluyendo bodega).
 * Salida: Instancia de SubtourCutGenerator inicializada.
 */
SubtourCutGenerator::SubtourCutGenerator(const Parser* parser, int numClients)
    : parserData(parser), n(numClients) {}

/*
 * Descripción: Método requerido por CBC para clonar el generador durante la 
 * distribución del árbol B&B.
 * Entrada: Ninguna.
 * Salida: Puntero a la nueva instancia clonada en el heap.
 */
CglCutGenerator* SubtourCutGenerator::clone() const {
    return new SubtourCutGenerator(parserData, n);
}

/*
 * Descripción: Detecta conjuntos de clientes que forman componentes inconexas 
 * (subtours) o que superan la capacidad vehicular.
 * Entrada: Arreglo de variables LP del nodo actual, umbral de tolerancia fraccional.
 * Salida: Vector de conjuntos inválidos (cada conjunto es un vector de IDs 0-indexados).
 */
vector<vector<int>> SubtourCutGenerator::findInvalidSets(const double* sol,
                                                         double threshold) const {
    vector<vector<int>> invalidSets;

    // ── 1. Construir lista de adyacencia del grafo de soporte ───────────────
    vector<vector<int>> adj(n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i != j && sol[getVarIndex(i, j)] > threshold) {
                adj[i].push_back(j);
            }
        }
    }

    // ── 2. BFS desde la bodega para identificar nodos alcanzables ───────────
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

    // ── 3. Agrupar nodos no alcanzables en componentes conexas (Subtours) ───
    vector<bool> processed(n, false);
    processed[0] = true; 

    for (int start = 1; start < n; start++) {
        if (reachable[start] || processed[start]) continue;

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

    // ── 4. Detectar violaciones de capacidad en rutas conectadas ────────────
    int Q = parserData->getCapacity();
    vector<bool> visitedInRoute(n, false);
    visitedInRoute[0] = true;

    for (int j = 1; j < n; j++) {
        if (sol[getVarIndex(0, j)] <= threshold) continue;
        if (visitedInRoute[j]) continue;

        vector<int> routeNodes;
        int totalDemand = 0;
        int curr = j;

        while (curr != 0 && !visitedInRoute[curr]) {
            visitedInRoute[curr] = true;
            routeNodes.push_back(curr);
            
            totalDemand += parserData->getClients()[curr + 1].getDemand();

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

        if (totalDemand > Q && !routeNodes.empty()) {
            invalidSets.push_back(routeNodes);
        }
    }

    return invalidSets;
}

/*
 * Descripción: Invocado automáticamente por CBC. Inserta restricciones DFJ y de 
 * capacidad basadas en las violaciones detectadas en la solución LP actual.
 * Entrada: Interfaz del solver, contenedor de cortes, información del árbol.
 * Salida: Ninguna (los cortes se insertan directamente en el contenedor 'cs').
 */
void SubtourCutGenerator::generateCuts(const OsiSolverInterface& si,
                                       OsiCuts& cs,
                                       const CglTreeInfo /*info*/) {
    const double* sol = si.getColSolution();
    if (!sol) return;

    int Q = parserData->getCapacity();

    // Análisis en dos pasadas: 
    // 1) threshold=0.5 para rutas cuasi-enteras.
    // 2) threshold=1e-4 para soluciones altamente fraccionales.
    const double thresholds[] = {0.5, 1e-4};
    const int numPasses = 2;

    for (int pass = 0; pass < numPasses; pass++) {
        double threshold = thresholds[pass];
        vector<vector<int>> invalidSets = findInvalidSets(sol, threshold);

        if (invalidSets.empty()) continue;

        for (const auto& S : invalidSets) {
            if (S.size() < 2) continue; 

            // Cálculo de demanda mínima de vehículos requerida (k_s)
            int demandSum = 0;
            for (int node : S) {
                if (node == 0) continue;
                demandSum += parserData->getClients()[node + 1].getDemand();
            }

            int k_s = static_cast<int>(std::ceil(static_cast<double>(demandSum) / Q));
            if (k_s < 1) k_s = 1;

            int rhs = static_cast<int>(S.size()) - k_s;
            if (rhs < 0) continue;

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

            OsiRowCut cut;
            cut.setRow(static_cast<int>(indices.size()),
                       indices.data(),
                       coefficients.data());
            cut.setLb(-1.0e30);          
            cut.setUb(static_cast<double>(rhs));

            // insertIfNotDuplicate previene colapsar la matriz con restricciones repetidas
            cs.insertIfNotDuplicate(cut);
        }
    }
}