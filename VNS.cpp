#include "VNS.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <map>
#include <climits>

using namespace std;

VNS::VNS(const Parser* parser) : parserData(parser) {}

// ─────────────────────────────────────────────────────────────
// Helpers de costo
// ─────────────────────────────────────────────────────────────

int VNS::insertionCost(int prev, int clientId, int next) const {
    // Costo de agregar clientId entre prev y next:
    //   d(prev, clientId) + d(clientId, next) - d(prev, next)
    return parserData->getDistance(prev, clientId)
         + parserData->getDistance(clientId, next)
         - parserData->getDistance(prev, next);
}

int VNS::removalSaving(int prev, int clientId, int next) const {
    // Ahorro de quitar clientId que estaba entre prev y next:
    //   d(prev, next) - d(prev, clientId) - d(clientId, next)
    return parserData->getDistance(prev, next)
         - parserData->getDistance(prev, clientId)
         - parserData->getDistance(clientId, next);
}

// ─────────────────────────────────────────────────────────────
// Vecindad 1 – Relocate
//
// Para cada cliente en cada ruta, evalúa moverlo a cada posición
// posible de cada otra ruta. Aplica el mejor movimiento encontrado
// (best-improvement) si mejora el costo total y no viola capacidad.
// ─────────────────────────────────────────────────────────────
bool VNS::neighborhoodRelocate(Solution& sol) {
    const auto& routes = sol.getRoutes();
    int numRoutes = routes.size();
    int Q = parserData->getCapacity();

    int bestDelta     = 0;   // Solo aceptamos mejoras estrictas (delta < 0)
    int bestFromRoute = -1;
    int bestFromPos   = -1;  // Posición del cliente en su ruta (1-indexado en path)
    int bestToRoute   = -1;
    int bestToPos     = -1;  // Posición de inserción en la ruta destino

    for (int r1 = 0; r1 < numRoutes; r1++) {
        const vector<int>& path1 = routes[r1].getPath();
        // path1[0] = bodega, path1[last] = bodega
        // Clientes están en índices 1..size-2
        int clientsInRoute = path1.size() - 2;
        if (clientsInRoute <= 1) continue; // No dejar ruta vacía

        for (int i = 1; i <= clientsInRoute; i++) {
            int client = path1[i];
            int prev1  = path1[i - 1];
            int next1  = path1[i + 1];
            int demand = parserData->getClients()[client].getDemand();

            // Ahorro de sacar este cliente de r1
            int saving = removalSaving(prev1, client, next1);

            for (int r2 = 0; r2 < numRoutes; r2++) {
                if (r1 == r2) continue;
                if (routes[r2].getCurrentLoad() + demand > Q) continue;

                const vector<int>& path2 = routes[r2].getPath();
                int positions = path2.size() - 1; // Posiciones válidas de inserción

                for (int j = 1; j <= positions; j++) {
                    int prevJ = path2[j - 1];
                    int nextJ = path2[j];

                    int cost  = insertionCost(prevJ, client, nextJ);
                    int delta = cost + saving; // Negativo = mejora

                    if (delta < bestDelta) {
                        bestDelta     = delta;
                        bestFromRoute = r1;
                        bestFromPos   = i;
                        bestToRoute   = r2;
                        bestToPos     = j;
                    }
                }
            }
        }
    }

    if (bestFromRoute == -1) return false; // Sin mejora

    // Aplicar el mejor movimiento encontrado reconstruyendo la solución
    // Necesitamos reconstruir desde los paths porque Solution/Route son inmutables
    // en tu diseño actual (no tienen setPath). Reconstruimos las rutas modificadas.
    Solution newSol(parserData);
    for (int r = 0; r < numRoutes; r++) {
        const vector<int>& oldPath = routes[r].getPath();
        Route newRoute(Q, parserData);

        if (r == bestFromRoute) {
            // Ruta origen: omitir el cliente en bestFromPos
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                if (i == bestFromPos) continue;
                newRoute.addClient(oldPath[i]);
            }
        } else if (r == bestToRoute) {
            // Ruta destino: insertar el cliente en bestToPos
            int movedClient = routes[bestFromRoute].getPath()[bestFromPos];
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                if (i == bestToPos) newRoute.addClient(movedClient);
                newRoute.addClient(oldPath[i]);
            }
            // Si bestToPos es al final (antes de la bodega de cierre)
            if (bestToPos == (int)oldPath.size() - 1) {
                newRoute.addClient(movedClient);
            }
        } else {
            // Ruta sin cambios
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                newRoute.addClient(oldPath[i]);
            }
        }
        // Solo agregar rutas no vacías
        if (newRoute.getPath().size() > 2) {
            newSol.addRoute(newRoute);
        }
    }

    if (newSol.isValid() && newSol.getTotalCost() < sol.getTotalCost()) {
        sol = newSol;
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
// Vecindad 2 – Swap
//
// Para cada par de clientes en rutas distintas, evalúa intercambiarlos.
// Aplica el mejor intercambio que mejore el costo y no viole capacidad.
// ─────────────────────────────────────────────────────────────
bool VNS::neighborhoodSwap(Solution& sol) {
    const auto& routes = sol.getRoutes();
    int numRoutes = routes.size();
    int Q = parserData->getCapacity();

    int bestDelta = 0;
    int bestR1 = -1, bestI = -1;
    int bestR2 = -1, bestJ = -1;

    for (int r1 = 0; r1 < numRoutes; r1++) {
        const vector<int>& path1 = routes[r1].getPath();
        int clients1 = path1.size() - 2;

        for (int i = 1; i <= clients1; i++) {
            int c1     = path1[i];
            int prev1  = path1[i - 1];
            int next1  = path1[i + 1];
            int dem1   = parserData->getClients()[c1].getDemand();

            for (int r2 = r1 + 1; r2 < numRoutes; r2++) {
                const vector<int>& path2 = routes[r2].getPath();
                int clients2 = path2.size() - 2;

                for (int j = 1; j <= clients2; j++) {
                    int c2     = path2[j];
                    int prev2  = path2[j - 1];
                    int next2  = path2[j + 1];
                    int dem2   = parserData->getClients()[c2].getDemand();

                    // Verificar capacidad con el intercambio
                    int newLoad1 = routes[r1].getCurrentLoad() - dem1 + dem2;
                    int newLoad2 = routes[r2].getCurrentLoad() - dem2 + dem1;
                    if (newLoad1 > Q || newLoad2 > Q) continue;

                    // Delta del swap: quitar c1 de r1 e insertar c2, y viceversa
                    int delta = 
                        // Costo nuevo en r1: c2 en lugar de c1
                        parserData->getDistance(prev1, c2) +
                        parserData->getDistance(c2, next1) -
                        parserData->getDistance(prev1, c1) -
                        parserData->getDistance(c1, next1) +
                        // Costo nuevo en r2: c1 en lugar de c2
                        parserData->getDistance(prev2, c1) +
                        parserData->getDistance(c1, next2) -
                        parserData->getDistance(prev2, c2) -
                        parserData->getDistance(c2, next2);

                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestR1 = r1; bestI = i;
                        bestR2 = r2; bestJ = j;
                    }
                }
            }
        }
    }

    if (bestR1 == -1) return false;

    // Aplicar el swap reconstruyendo las dos rutas afectadas
    int c1 = routes[bestR1].getPath()[bestI];
    int c2 = routes[bestR2].getPath()[bestJ];

    Solution newSol(parserData);
    for (int r = 0; r < numRoutes; r++) {
        const vector<int>& oldPath = routes[r].getPath();
        Route newRoute(Q, parserData);

        for (int i = 1; i < (int)oldPath.size() - 1; i++) {
            if (r == bestR1 && i == bestI)      newRoute.addClient(c2);
            else if (r == bestR2 && i == bestJ) newRoute.addClient(c1);
            else                                newRoute.addClient(oldPath[i]);
        }
        if (newRoute.getPath().size() > 2) newSol.addRoute(newRoute);
    }

    if (newSol.isValid() && newSol.getTotalCost() < sol.getTotalCost()) {
        sol = newSol;
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
// Vecindad 3/4 – Or-Opt con segmento de largo `segLen`
//
// Toma un segmento de `segLen` clientes consecutivos de una ruta
// y evalúa moverlo completo a la mejor posición en cualquier otra
// ruta que tenga capacidad suficiente.
// ─────────────────────────────────────────────────────────────
bool VNS::neighborhoodOrOpt(Solution& sol, int segLen) {
    const auto& routes = sol.getRoutes();
    int numRoutes = routes.size();
    int Q = parserData->getCapacity();

    int bestDelta   = 0;
    int bestR1      = -1, bestSegStart = -1;
    int bestR2      = -1, bestInsertPos = -1;

    for (int r1 = 0; r1 < numRoutes; r1++) {
        const vector<int>& path1 = routes[r1].getPath();
        int clients1 = path1.size() - 2;
        if (clients1 <= segLen) continue; // No dejar ruta vacía

        for (int i = 1; i <= clients1 - segLen + 1; i++) {
            // Segmento: path1[i .. i+segLen-1]
            int segDemand = 0;
            for (int s = 0; s < segLen; s++)
                segDemand += parserData->getClients()[path1[i + s]].getDemand();

            int prevSeg = path1[i - 1];
            int nextSeg = path1[i + segLen]; // Nodo que queda tras el segmento
            int segFirst = path1[i];
            int segLast  = path1[i + segLen - 1];

            // Ahorro de retirar el segmento de r1
            int saving = parserData->getDistance(prevSeg, nextSeg)
                       - parserData->getDistance(prevSeg, segFirst)
                       - parserData->getDistance(segLast, nextSeg);

            for (int r2 = 0; r2 < numRoutes; r2++) {
                if (r1 == r2) continue;
                if (routes[r2].getCurrentLoad() + segDemand > Q) continue;

                const vector<int>& path2 = routes[r2].getPath();
                int positions = path2.size() - 1;

                for (int j = 1; j <= positions; j++) {
                    int prevJ = path2[j - 1];
                    int nextJ = path2[j];

                    int insertCost = parserData->getDistance(prevJ, segFirst)
                                   + parserData->getDistance(segLast, nextJ)
                                   - parserData->getDistance(prevJ, nextJ);

                    int delta = insertCost + saving; // saving es negativo

                    if (delta < bestDelta) {
                        bestDelta    = delta;
                        bestR1       = r1; bestSegStart  = i;
                        bestR2       = r2; bestInsertPos = j;
                    }
                }
            }
        }
    }

    if (bestR1 == -1) return false;

    // Recopilar el segmento a mover
    const vector<int>& srcPath = routes[bestR1].getPath();
    vector<int> segment;
    for (int s = 0; s < segLen; s++)
        segment.push_back(srcPath[bestSegStart + s]);

    Solution newSol(parserData);
    for (int r = 0; r < numRoutes; r++) {
        const vector<int>& oldPath = routes[r].getPath();
        Route newRoute(Q, parserData);

        if (r == bestR1) {
            // Omitir el segmento
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                if (i >= bestSegStart && i < bestSegStart + segLen) continue;
                newRoute.addClient(oldPath[i]);
            }
        } else if (r == bestR2) {
            // Insertar segmento en bestInsertPos
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                if (i == bestInsertPos)
                    for (int c : segment) newRoute.addClient(c);
                newRoute.addClient(oldPath[i]);
            }
            if (bestInsertPos == (int)oldPath.size() - 1)
                for (int c : segment) newRoute.addClient(c);
        } else {
            for (int i = 1; i < (int)oldPath.size() - 1; i++)
                newRoute.addClient(oldPath[i]);
        }
        if (newRoute.getPath().size() > 2) newSol.addRoute(newRoute);
    }

    if (newSol.isValid() && newSol.getTotalCost() < sol.getTotalCost()) {
        sol = newSol;
        return true;
    }
    return false;
}

// shake
//
// Extrae `intensity` clientes aleatorios de sus rutas y los
// reinserta en la mejor posición factible disponible (Best
// Insertion). Esto rompe rutas densamente cargadas y crea
// capacidad para que los operadores de descenso puedan operar.
// ─────────────────────────────────────────────────────────────
Solution VNS::shake(const Solution& sol, int intensity, mt19937& rng) const {
    int Q = parserData->getCapacity();

    // Construir lista de todos los clientes y de qué ruta vienen
    // Trabajamos con copias de los paths para poder modificarlos
    vector<vector<int>> paths;
    for (const auto& route : sol.getRoutes())
        paths.push_back(route.getPath());

    int numRoutes = paths.size();

    // Recopilar todos los clientes disponibles para extracción
    // (excluimos la bodega, índice 0 en path = nodo 1)
    vector<pair<int,int>> allClients; // (routeIdx, posInPath)
    for (int r = 0; r < numRoutes; r++) {
        int routeSize = paths[r].size() - 2; // sin las dos bodegas
        // Solo extraer de rutas con más de 1 cliente para no dejarlas vacías
        if (routeSize > 1) {
            for (int i = 1; i <= routeSize; i++)
                allClients.push_back({r, i});
        }
    }

    if (allClients.empty()) return sol;

    // Limitar intensity al máximo disponible
    intensity = min(intensity, (int)allClients.size());

    // Seleccionar `intensity` clientes aleatorios para extraer
    shuffle(allClients.begin(), allClients.end(), rng);
    vector<int> extracted; // IDs de clientes extraídos

    // Extraer de atrás hacia adelante para no invalidar índices
    // Agrupar por ruta y ordenar posiciones descendentes
    map<int, vector<int>> toRemoveByRoute;
    for (int k = 0; k < intensity; k++)
        toRemoveByRoute[allClients[k].first].push_back(allClients[k].second);

    for (auto& [routeIdx, positions] : toRemoveByRoute) {
        sort(positions.rbegin(), positions.rend()); // mayor índice primero
        for (int pos : positions) {
            extracted.push_back(paths[routeIdx][pos]);
            paths[routeIdx].erase(paths[routeIdx].begin() + pos);
        }
    }

    // Reinsertar cada cliente extraído en la mejor posición factible
    for (int clientId : extracted) {
        int demand = parserData->getClients()[clientId].getDemand();
        int bestDelta = INT_MAX;
        int bestRoute = -1;
        int bestPos   = -1;

        for (int r = 0; r < numRoutes; r++) {
            // Calcular carga actual de la ruta
            int load = 0;
            for (int node : paths[r])
                if (node != 1) load += parserData->getClients()[node].getDemand();

            if (load + demand > Q) continue;

            int pathSize = paths[r].size();
            for (int i = 1; i < pathSize; i++) {
                int prev = paths[r][i - 1];
                int next = paths[r][i];
                int delta = parserData->getDistance(prev, clientId)
                          + parserData->getDistance(clientId, next)
                          - parserData->getDistance(prev, next);
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestRoute = r;
                    bestPos   = i;
                }
            }
        }

        if (bestRoute != -1) {
            paths[bestRoute].insert(paths[bestRoute].begin() + bestPos, clientId);
        } else {
            // Si no cabe en ninguna ruta existente, crear ruta nueva
            paths.push_back({1, clientId, 1});
            numRoutes++;
        }
    }

    // Reconstruir solución desde los paths modificados
    Solution newSol(parserData);
    for (const auto& path : paths) {
        if (path.size() <= 2) continue; // Ruta vacía
        Route newRoute(Q, parserData);
        for (int i = 1; i < (int)path.size() - 1; i++)
            newRoute.addClient(path[i]);
        newSol.addRoute(newRoute);
    }

    return newSol;
}


// ─────────────────────────────────────────────────────────────
// optimize – Bucle principal VNS
//
// Esquema VND (Variable Neighborhood Descent):
//   k = 0 (Relocate)
//   mientras k < NUM_VECINDADES:
//     si vecindad k mejora → reiniciar k = 0
//     si no mejora         → k++
//   tras cada mejora inter-ruta, aplicar 3-OPT intra-ruta
// ─────────────────────────────────────────────────────────────
Solution VNS::optimize(const Solution& initialSolution) {
    KOpt kopt(parserData);
    mt19937 rng(42); // Semilla fija para reproducibilidad

    // Parámetros VNS
    const int K_MAX        = 5;   // Máximo nivel de shaking
    const int MAX_ITER     = 100; // Iteraciones sin mejora antes de parar
    // Intensidad de shaking: extraer entre 2 y 2+K clientes
    const int BASE_SHAKE   = 2;

    // ── Fase 1: VND inicial ──
    Solution best = kopt.optimize(initialSolution);
    Solution current = best;

    // VND sobre la solución inicial
    bool improved = true;
    while (improved) {
        improved = false;
        if (neighborhoodRelocate(current)) { current = kopt.optimize(current); improved = true; continue; }
        if (neighborhoodSwap(current))     { current = kopt.optimize(current); improved = true; continue; }
        if (neighborhoodOrOpt(current, 2)) { current = kopt.optimize(current); improved = true; continue; }
        if (neighborhoodOrOpt(current, 3)) { current = kopt.optimize(current); improved = true; continue; }
    }
    best = current;

    // ── Fase 2: Loop VNS con shaking ──
    int k       = 1;
    int noImproveCount = 0;

    while (noImproveCount < MAX_ITER) {
        // Shaking: perturbar la mejor solución conocida
        int intensity = BASE_SHAKE + k;
        Solution shaken = shake(best, intensity, rng);

        // VND sobre la solución perturbada
        Solution candidate = shaken;
        improved = true;
        while (improved) {
            improved = false;
            if (neighborhoodRelocate(candidate)) { candidate = kopt.optimize(candidate); improved = true; continue; }
            if (neighborhoodSwap(candidate))     { candidate = kopt.optimize(candidate); improved = true; continue; }
            if (neighborhoodOrOpt(candidate, 2)) { candidate = kopt.optimize(candidate); improved = true; continue; }
            if (neighborhoodOrOpt(candidate, 3)) { candidate = kopt.optimize(candidate); improved = true; continue; }
        }

        // ¿Mejoró la mejor solución conocida?
        if (candidate.getTotalCost() < best.getTotalCost()) {
            best    = candidate;
            k       = 1;          // Reiniciar con perturbación mínima
            noImproveCount = 0;
        } else {
            k = (k % K_MAX) + 1;  // Aumentar intensidad cíclicamente
            noImproveCount++;
        }
    }

    return best;
}