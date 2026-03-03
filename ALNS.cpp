#include "ALNS.h"
#include "CbcSolver.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cassert>

using namespace std;
using chrono::steady_clock;
using chrono::duration;

/*
 * Descripción: Constructor de la metaheurística ALNS.
 * Entrada: Puntero a la instancia del parser, puntero al solver exacto CBC.
 * Salida: Instancia de la clase inicializada.
 */
ALNS::ALNS(const Parser* parser, CbcSolver* cbc)
    : parserData(parser), cbcSolver(cbc), vns(parser), rng(42) {}

/*
 * Descripción: Bucle principal de optimización. Alterna heurísticas de destrucción con reparación 
 * exacta MIP. Utiliza un esquema de aceptación basado en Recocido Simulado (SA) 
 * para escapar de óptimos locales.
 * Entrada: Solución base (initialSol), límite de tiempo de ejecución en segundos.
 * Salida: La mejor solución factible encontrada durante el ciclo.
 */
Solution ALNS::optimize(const Solution& initialSol, double timeLimitSeconds) {
    auto startTime = steady_clock::now();
    double bigM = initialSol.getTotalCost() * 0.5;
    
    Solution bestSol    = initialSol;
    Solution currentSol = initialSol;

    int iteration = 0;

    vector<double> weights   = {1.0, 1.0, 1.0};
    vector<int>    successes = {0,   0,   0  };
    vector<int>    attempts  = {0,   0,   0  };

    double T_start = initialSol.getTotalCost() * 0.05; 
    double T_end   = 0.1;

    int currentDestroySize = 6; 
    const int maxDestroySize = min(16, parserData->getDimension() / 4);
    int noImprovementCounter = 0;
    const int MAX_NO_IMPROVE = 15; 

    cout << "[ALNS] Inicio | Costo: " << bestSol.getTotalCost()
         << " | destroySize dinamico: " << currentDestroySize << " a " << maxDestroySize << endl;

    while (true) {
        double elapsed = duration<double>(steady_clock::now() - startTime).count();
        if (elapsed >= timeLimitSeconds) break;

        iteration++;

        // ── 1. Seleccionar operador destroy adaptativamente ──────────────
        double totalWeight = weights[0] + weights[1] + weights[2];
        double r = uniform_real_distribution<double>(0.0, totalWeight)(rng);

        int opIdx;
        if      (r < weights[0])               opIdx = 0; 
        else if (r < weights[0] + weights[1])  opIdx = 1; 
        else                                   opIdx = 2; 

        attempts[opIdx]++;

        // ── 2. Destroy ───────────────────────────────────────────────────
        vector<int> removed;
        switch (opIdx) {
            case 0: removed = shawRemoval(currentSol, currentDestroySize); break;
            case 1: removed = worstRemoval(currentSol, currentDestroySize); break;
            case 2: removed = randomRemoval(currentSol, currentDestroySize); break;
        }

        if (removed.empty()) continue;

        // ── 3. CBC Repair ────────────────────────────────────────────────
        double timeLeft = timeLimitSeconds - elapsed;
        double subproblemLimit = min(2.0, timeLeft * 0.1); 

        Solution repairedSol = cbcSolver->solveSubproblem(currentSol, removed, subproblemLimit);

        if (!repairedSol.isValid()) continue;

        // ── 4. VNS Polish ────────────────────────────────────────────────
        Solution polishedSol = vns.optimize(repairedSol, vnsIter);

        // ── 5. Aceptación (Simulated Annealing) ──────────────────────────
        double currentCost = currentSol.getTotalCost();
        
        Solution cleanedSol(parserData);
        for (const auto& route : polishedSol.getRoutes()) {
            if (route.getPath().size() > 2) {
                cleanedSol.addRoute(route);
            }
        }
        
        double newCost = cleanedSol.getTotalCost();

        // Castigo matemático para descartar ramificaciones que rompan el Bin Packing
        int baseK = initialSol.getRoutes().size();
        int newK  = cleanedSol.getRoutes().size();
        if (newK > baseK) {
            newCost += bigM * (newK - baseK); 
        }

        double delta = newCost - currentCost;
        double coolingHorizon = min(50.0, timeLimitSeconds * 0.8);
        double progress = min(1.0, elapsed / coolingHorizon);
        double T = T_start * pow(T_end / T_start, progress);

        bool accept = false;
        if (delta < -0.01) {
            accept = true;
        } else {
            double p = exp(-delta / T);
            double prob_r = uniform_real_distribution<double>(0.0, 1.0)(rng);
            if (prob_r < p) accept = true;
        }

        if (accept) {
            currentSol = cleanedSol;
            successes[opIdx]++; 

            if (newCost < bestSol.getTotalCost() - 0.01) {
                bestSol = cleanedSol;
                currentDestroySize = 6; 
                noImprovementCounter = 0;
                cout << "EN " << elapsed << "s" "   ---> [NUEVO OPTIMO GLOBAL]: " << bestSol.getTotalCost() << "\n";
            } else {
                noImprovementCounter++; 
            }
        } else {
            noImprovementCounter++; 
        }

        if (noImprovementCounter >= MAX_NO_IMPROVE) {
            currentDestroySize = min(maxDestroySize, currentDestroySize + 3);
            noImprovementCounter = 0; 
        }

        // ── 6. Actualizar pesos adaptativos ──────────
        if (iteration % 20 == 0) {
            for (int i = 0; i < 3; i++) {
                if (attempts[i] > 0) {
                    double successRate = (double)successes[i] / attempts[i];
                    weights[i] = 0.8 * weights[i] + 0.2 * (successRate + 0.1);
                }
                successes[i] = 0;
                attempts[i]  = 0;
            }
        }
    }

    double totalTime = duration<double>(steady_clock::now() - startTime).count();
    cout << "[ALNS] Fin | Mejor costo: " << bestSol.getTotalCost()
         << " | Iteraciones: " << iteration
         << " | Tiempo: " << totalTime << "s" << endl;

    return bestSol;
}

/*
 * Descripción: Operador de destrucción espacial. Extrae un cliente aleatorio y sus vecinos más cercanos.
 * Entrada: Solución actual, cantidad 'k' de nodos a extraer.
 * Salida: Vector con los IDs de los clientes removidos.
 */
vector<int> ALNS::shawRemoval(const Solution& sol, int k) {
    const auto& routes = sol.getRoutes();
    vector<int> allClients;
    
    for (const auto& route : routes) {
        const auto& path = route.getPath();
        for (size_t i = 1; i < path.size() - 1; ++i) {
            allClients.push_back(path[i]);
        }
    }

    if (allClients.empty()) return {};
    k = min(k, (int)allClients.size());

    int seedIdx  = uniform_int_distribution<int>(0, allClients.size() - 1)(rng);
    int seedClient = allClients[seedIdx];

    vector<pair<double,int>> byDistance;
    for (int c : allClients) {
        if (c == seedClient) continue;
        double d = parserData->getDistance(seedClient, c);
        byDistance.push_back({d, c});
    }
    sort(byDistance.begin(), byDistance.end());

    vector<int> removed = {seedClient};
    for (int i = 0; i < k - 1 && i < (int)byDistance.size(); ++i) {
        removed.push_back(byDistance[i].second);
    }

    return removed;
}

/*
 * Descripción: Operador de destrucción por costo. Extrae los nodos que presentan la peor 
 * eficiencia de ruteo (mayor ahorro al ser removidos).
 * Entrada: Solución actual, cantidad 'k' de nodos a extraer.
 * Salida: Vector con los IDs de los clientes removidos.
 */
vector<int> ALNS::worstRemoval(const Solution& sol, int k) {
    vector<pair<double,int>> savings;

    for (const auto& route : sol.getRoutes()) {
        const auto& path = route.getPath();
        for (size_t i = 1; i < path.size() - 1; ++i) {
            int prev   = path[i-1];
            int curr   = path[i];
            int next   = path[i+1];
            double s   = removalSavings(curr, prev, next);
            savings.push_back({s, curr});
        }
    }

    sort(savings.begin(), savings.end(),
         [](const pair<double,int>& a, const pair<double,int>& b){
             return a.first > b.first;
         });

    k = min(k, (int)savings.size());
    vector<int> removed;
    for (int i = 0; i < k; ++i) {
        removed.push_back(savings[i].second);
    }

    return removed;
}

/*
 * Descripción: Operador de destrucción aleatoria para favorecer la diversificación del espacio de búsqueda.
 * Entrada: Solución actual, cantidad 'k' de nodos a extraer.
 * Salida: Vector con los IDs de los clientes removidos.
 */
vector<int> ALNS::randomRemoval(const Solution& sol, int k) {
    vector<int> allClients;
    
    for (const auto& route : sol.getRoutes()) {
        const auto& path = route.getPath();
        for (size_t i = 1; i < path.size() - 1; ++i) {
            allClients.push_back(path[i]);
        }
    }

    if (allClients.empty()) return {};
    k = min(k, (int)allClients.size());

    shuffle(allClients.begin(), allClients.end(), rng);
    return vector<int>(allClients.begin(), allClients.begin() + k);
}

/*
 * Descripción: Evalúa la variación en la función objetivo al insertar un nodo entre otros dos.
 * Entrada: ID del cliente a evaluar, ID del cliente previo, ID del cliente siguiente.
 * Salida: Variación del costo.
 */
double ALNS::insertionCost(int clientId, int prev, int next) const {
    return parserData->getDistance(prev, clientId)
         + parserData->getDistance(clientId, next)
         - parserData->getDistance(prev, next);
}

/*
 * Descripción: Evalúa el ahorro producido al quitar un nodo de su posición actual.
 * Entrada: ID del cliente a evaluar, ID del cliente previo, ID del cliente siguiente.
 * Salida: Ahorro de la eliminación.
 */
double ALNS::removalSavings(int clientId, int prev, int next) const {
    return parserData->getDistance(prev, clientId)
         + parserData->getDistance(clientId, next)
         - parserData->getDistance(prev, next);
}