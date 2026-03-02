#include "VNS.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <map>
#include <climits>

using namespace std;

/*
 * Descripción: Constructor de la metaheurística VNS.
 * Entrada: Puntero constante al parser con los datos de la instancia.
 * Salida: Instancia inicializada.
 */
VNS::VNS(const Parser* parser) : parserData(parser) {}

// ─────────────────────────────────────────────────────────────
// Métodos Auxiliares de Costo
// ─────────────────────────────────────────────────────────────

/*
 * Descripción: Calcula la variación en la función objetivo al insertar un cliente 
 * específico entre dos nodos adyacentes.
 * Entrada: ID del nodo previo, ID del cliente a insertar, ID del nodo posterior.
 * Salida: Entero representando el costo adicional generado por la inserción.
 */
int VNS::insertionCost(int prev, int clientId, int next) const {
    return parserData->getDistance(prev, clientId)
         + parserData->getDistance(clientId, next)
         - parserData->getDistance(prev, next);
}

/*
 * Descripción: Calcula el ahorro en la función objetivo al remover un cliente 
 * que se encuentra entre dos nodos adyacentes.
 * Entrada: ID del nodo previo, ID del cliente a remover, ID del nodo posterior.
 * Salida: Entero representando la distancia ahorrada (positiva).
 */
int VNS::removalSaving(int prev, int clientId, int next) const {
    return parserData->getDistance(prev, next)
         - parserData->getDistance(prev, clientId)
         - parserData->getDistance(clientId, next);
}

// ─────────────────────────────────────────────────────────────
// Estructuras de Vecindad
// ─────────────────────────────────────────────────────────────

/*
 * Descripción: Explora la vecindad "Relocate". Evalúa mover un cliente de su ruta actual 
 * a la mejor posición posible dentro de cualquier otra ruta distinta. 
 * Aplica el mejor movimiento encontrado (Best-Improvement) que respete las capacidades.
 * Entrada: Solución actual por referencia.
 * Salida: Booleano indicando si se aplicó un movimiento que mejora estrictamente el costo.
 */
bool VNS::neighborhoodRelocate(Solution& sol) {
    const auto& routes = sol.getRoutes();
    int numRoutes = routes.size();
    int Q = parserData->getCapacity();

    int bestDelta     = 0;   
    int bestFromRoute = -1;
    int bestFromPos   = -1;  
    int bestToRoute   = -1;
    int bestToPos     = -1;  

    for (int r1 = 0; r1 < numRoutes; r1++) {
        const vector<int>& path1 = routes[r1].getPath();
        int clientsInRoute = path1.size() - 2;
        if (clientsInRoute <= 1) continue; 

        for (int i = 1; i <= clientsInRoute; i++) {
            int client = path1[i];
            int prev1  = path1[i - 1];
            int next1  = path1[i + 1];
            int demand = parserData->getClients()[client].getDemand();

            int saving = removalSaving(prev1, client, next1);

            for (int r2 = 0; r2 < numRoutes; r2++) {
                if (r1 == r2) continue;
                if (routes[r2].getCurrentLoad() + demand > Q) continue;

                const vector<int>& path2 = routes[r2].getPath();
                int positions = path2.size() - 1; 

                for (int j = 1; j <= positions; j++) {
                    int prevJ = path2[j - 1];
                    int nextJ = path2[j];

                    int cost  = insertionCost(prevJ, client, nextJ);
                    int delta = cost + saving; 

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

    if (bestFromRoute == -1) return false; 

    Solution newSol(parserData);
    for (int r = 0; r < numRoutes; r++) {
        const vector<int>& oldPath = routes[r].getPath();
        Route newRoute(Q, parserData);

        if (r == bestFromRoute) {
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                if (i == bestFromPos) continue;
                newRoute.addClient(oldPath[i]);
            }
        } else if (r == bestToRoute) {
            int movedClient = routes[bestFromRoute].getPath()[bestFromPos];
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                if (i == bestToPos) newRoute.addClient(movedClient);
                newRoute.addClient(oldPath[i]);
            }
            if (bestToPos == (int)oldPath.size() - 1) {
                newRoute.addClient(movedClient);
            }
        } else {
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                newRoute.addClient(oldPath[i]);
            }
        }
        
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

/*
 * Descripción: Explora la vecindad "Swap". Evalúa intercambiar la posición de dos clientes 
 * pertenecientes a rutas distintas. Aplica el intercambio que genere la mayor reducción 
 * en la función objetivo sin violar restricciones de capacidad.
 * Entrada: Solución actual por referencia.
 * Salida: Booleano indicando si se logró una mejora estricta.
 */
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

                    int newLoad1 = routes[r1].getCurrentLoad() - dem1 + dem2;
                    int newLoad2 = routes[r2].getCurrentLoad() - dem2 + dem1;
                    if (newLoad1 > Q || newLoad2 > Q) continue;

                    int delta = 
                        parserData->getDistance(prev1, c2) +
                        parserData->getDistance(c2, next1) -
                        parserData->getDistance(prev1, c1) -
                        parserData->getDistance(c1, next1) +
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

/*
 * Descripción: Explora la vecindad "Or-Opt". Extrae un segmento de tamaño 'segLen' 
 * de una ruta y evalúa su inserción completa en la mejor posición disponible 
 * de cualquier otra ruta de la flota.
 * Entrada: Solución actual por referencia, longitud del segmento a mover (usualmente 2 o 3).
 * Salida: Booleano indicando si se logró una mejora estricta.
 */
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
        if (clients1 <= segLen) continue; 

        for (int i = 1; i <= clients1 - segLen + 1; i++) {
            int segDemand = 0;
            for (int s = 0; s < segLen; s++)
                segDemand += parserData->getClients()[path1[i + s]].getDemand();

            int prevSeg = path1[i - 1];
            int nextSeg = path1[i + segLen]; 
            int segFirst = path1[i];
            int segLast  = path1[i + segLen - 1];

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

                    int delta = insertCost + saving; 

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

    const vector<int>& srcPath = routes[bestR1].getPath();
    vector<int> segment;
    for (int s = 0; s < segLen; s++)
        segment.push_back(srcPath[bestSegStart + s]);

    Solution newSol(parserData);
    for (int r = 0; r < numRoutes; r++) {
        const vector<int>& oldPath = routes[r].getPath();
        Route newRoute(Q, parserData);

        if (r == bestR1) {
            for (int i = 1; i < (int)oldPath.size() - 1; i++) {
                if (i >= bestSegStart && i < bestSegStart + segLen) continue;
                newRoute.addClient(oldPath[i]);
            }
        } else if (r == bestR2) {
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

// ─────────────────────────────────────────────────────────────
// Ciclo Principal de Optimización
// ─────────────────────────────────────────────────────────────

/*
 * Descripción: Aplica una fase de agitación (Shaking) a la solución. Extrae aleatoriamente 
 * una cantidad determinada ('intensity') de clientes y los reinserta empleando heurística 
 * Greedy. Este mecanismo permite escapar de valles de óptimos locales profundos.
 * Entrada: Solución base actual, nivel de intensidad de la agitación, generador pseudo-aleatorio.
 * Salida: Nueva solución perturbada.
 */
Solution VNS::shake(const Solution& sol, int intensity, mt19937& rng) const {
    int Q = parserData->getCapacity();

    vector<vector<int>> paths;
    for (const auto& route : sol.getRoutes())
        paths.push_back(route.getPath());

    int numRoutes = paths.size();

    vector<pair<int,int>> allClients; 
    for (int r = 0; r < numRoutes; r++) {
        int routeSize = paths[r].size() - 2; 
        if (routeSize > 1) {
            for (int i = 1; i <= routeSize; i++)
                allClients.push_back({r, i});
        }
    }

    if (allClients.empty()) return sol;

    intensity = min(intensity, (int)allClients.size());

    shuffle(allClients.begin(), allClients.end(), rng);
    vector<int> extracted; 

    map<int, vector<int>> toRemoveByRoute;
    for (int k = 0; k < intensity; k++)
        toRemoveByRoute[allClients[k].first].push_back(allClients[k].second);

    for (auto& [routeIdx, positions] : toRemoveByRoute) {
        sort(positions.rbegin(), positions.rend()); 
        for (int pos : positions) {
            extracted.push_back(paths[routeIdx][pos]);
            paths[routeIdx].erase(paths[routeIdx].begin() + pos);
        }
    }

    for (int clientId : extracted) {
        int demand = parserData->getClients()[clientId].getDemand();
        int bestDelta = INT_MAX;
        int bestRoute = -1;
        int bestPos   = -1;

        for (int r = 0; r < numRoutes; r++) {
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
            paths.push_back({1, clientId, 1});
            numRoutes++;
        }
    }

    Solution newSol(parserData);
    for (const auto& path : paths) {
        if (path.size() <= 2) continue; 
        Route newRoute(Q, parserData);
        for (int i = 1; i < (int)path.size() - 1; i++)
            newRoute.addClient(path[i]);
        newSol.addRoute(newRoute);
    }

    return newSol;
}

/*
 * Descripción: Función principal del algoritmo VNS. Alterna sistemáticamente entre las vecindades 
 * iterando bajo el esquema Variable Neighborhood Descent (VND). Emplea 3-OPT como post-optimización 
 * y mecanismos de Shaking si se estanca.
 * Entrada: Solución factible inicial, cantidad máxima de iteraciones sin mejora permitidas.
 * Salida: Mejor solución local/global encontrada.
 */
Solution VNS::optimize(const Solution& initialSolution, int maxIter) {
    KOpt kopt(parserData);
    mt19937 rng(42); 

    const int K_MAX        = 5;   
    const int MAX_ITER     = maxIter; 
    const int BASE_SHAKE   = 2;

    // ── 1. Fase Inicial: VND sin perturbación ──
    Solution best = kopt.optimize(initialSolution);
    Solution current = best;

    bool improved = true;
    while (improved) {
        improved = false;
        if (neighborhoodRelocate(current)) { current = kopt.optimize(current); improved = true; continue; }
        if (neighborhoodSwap(current))     { current = kopt.optimize(current); improved = true; continue; }
        if (neighborhoodOrOpt(current, 2)) { current = kopt.optimize(current); improved = true; continue; }
        if (neighborhoodOrOpt(current, 3)) { current = kopt.optimize(current); improved = true; continue; }
    }
    best = current;

    // ── 2. Fase de Exploración: Loop VNS + Shaking ──
    int k       = 1;
    int noImproveCount = 0;

    while (noImproveCount < MAX_ITER) {
        int intensity = BASE_SHAKE + k;
        Solution shaken = shake(best, intensity, rng);

        Solution candidate = shaken;
        improved = true;
        while (improved) {
            improved = false;
            if (neighborhoodRelocate(candidate)) { candidate = kopt.optimize(candidate); improved = true; continue; }
            if (neighborhoodSwap(candidate))     { candidate = kopt.optimize(candidate); improved = true; continue; }
            if (neighborhoodOrOpt(candidate, 2)) { candidate = kopt.optimize(candidate); improved = true; continue; }
            if (neighborhoodOrOpt(candidate, 3)) { candidate = kopt.optimize(candidate); improved = true; continue; }
        }

        if (candidate.getTotalCost() < best.getTotalCost()) {
            best    = candidate;
            k       = 1;          
            noImproveCount = 0;
        } else {
            k = (k % K_MAX) + 1;  
            noImproveCount++;
        }
    }

    return best;
}