#include "ALNS.h"
#include "CbcSolver.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cassert>

using namespace std;
using chrono::steady_clock;
using chrono::duration;

ALNS::ALNS(const Parser* parser, CbcSolver* cbc)
    : parserData(parser), cbcSolver(cbc), vns(parser),
      rng(42) // seed fija para reproducibilidad
{}

// =============================================================================
// optimize
// =============================================================================
Solution ALNS::optimize(const Solution& initialSol, double timeLimitSeconds) {
    auto startTime = steady_clock::now();

    Solution bestSol    = initialSol;
    Solution currentSol = initialSol;

    int iteration = 0;

    // Pesos adaptativos de los operadores
    vector<double> weights   = {1.0, 1.0, 1.0};
    vector<int>    successes = {0,   0,   0  };
    vector<int>    attempts  = {0,   0,   0  };

    // Parámetros para Simulated Annealing atado al reloj
    double T_start = initialSol.getTotalCost() * 0.05; 
    double T_end   = 0.1;

    // --- NUEVO: Escalada Dinámica de Destrucción ---
    int currentDestroySize = 6; // Empezamos con subproblemas pequeñitos y ultrarrápidos
    const int maxDestroySize = min(16, parserData->getDimension() / 4); // Cota máxima
    int noImprovementCounter = 0;
    const int MAX_NO_IMPROVE = 15; // Iteraciones de paciencia antes de subir la destrucción

    cout << "[ALNS] Inicio | Costo: " << bestSol.getTotalCost()
         << " | destroySize dinamico: " << currentDestroySize << " a " << maxDestroySize << endl;

    while (true) {
        // Verificar tiempo
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

        // ── 2. Destroy (usando el tamaño dinámico) ───────────────────────
        vector<int> removed;
        switch (opIdx) {
            case 0: removed = shawRemoval  (currentSol, currentDestroySize); break;
            case 1: removed = worstRemoval (currentSol, currentDestroySize); break;
            case 2: removed = randomRemoval(currentSol, currentDestroySize); break;
        }

        if (removed.empty()) continue;

        // ── 3. CBC Repair ─────────────────────────────────────────────────
        double timeLeft = timeLimitSeconds - elapsed;
        double subproblemLimit = min(2.0, timeLeft * 0.1); 

        Solution repairedSol = cbcSolver->solveSubproblem(currentSol, removed, subproblemLimit);

        if (!repairedSol.isValid()) continue;

        // ── 4. VNS Polish ─────────────────────────────────────────────────
        Solution polishedSol = vns.optimize(repairedSol, vnsIter);

        // ── 5. Aceptación (Simulated Annealing) ───────────────────────────
        double currentCost   = currentSol.getTotalCost();
        double repairedCost  = repairedSol.getTotalCost();
        
        // Filtrar rutas vacías que el VNS haya podido dejar
        Solution cleanedSol(parserData);
        for (const auto& r : polishedSol.getRoutes()) {
            if (r.getPath().size() > 2) cleanedSol.addRoute(r);
        }
        
        double newCost = cleanedSol.getTotalCost();

        // --- PENALIZACIÓN POR INFLACIÓN DE FLOTA (BIG M) ---
        int baseK = initialSol.getRoutes().size();
        int newK  = cleanedSol.getRoutes().size();
        if (newK > baseK) {
            newCost += 5000.0 * (newK - baseK); // Destruye el costo si agrega camiones
        }

        double delta = newCost - currentCost;

        double progress = elapsed / timeLimitSeconds; 
        double T = T_start * pow(T_end / T_start, progress);

        bool accept = false;
        double p = 0.0, prob_r = 0.0;

        if (delta < -0.01) {
            accept = true;
        } else {
            p = exp(-delta / T);
            prob_r = uniform_real_distribution<double>(0.0, 1.0)(rng);
            if (prob_r < p) accept = true;
        }

        // --- DEBUG PRINT (TRAZABILIDAD POR ITERACIÓN) ---
        cout << "[ALNS Debug] It: " << iteration 
             << " | Op: " << opIdx 
             << " | Destr: " << currentDestroySize
             << " | Base: " << currentCost 
             << " | Repaired(CBC): " << repairedCost 
             << " | Polished(VNS): " << cleanedSol.getTotalCost() 
             << " | K: " << newK
             << " | Delta: " << delta;

        if (delta < -0.01) {
            cout << " | ACCEPT (Mejora estricta)\\n";
        } else {
            cout << " | T: " << T << " | P: " << p << " | R: " << prob_r;
            if (accept) cout << " | ACCEPT (SA)\\n";
            else        cout << " | REJECT\\n";
        }
        // ------------------------------------------------

        if (accept) {
            currentSol = cleanedSol;
            successes[opIdx]++; 

            if (newCost < bestSol.getTotalCost() - 0.01) {
                bestSol = cleanedSol;
                currentDestroySize = 6; 
                noImprovementCounter = 0;
                cout << "   ---> [NUEVO OPTIMO GLOBAL]: " << bestSol.getTotalCost() << "\\n";
            } else {
                noImprovementCounter++; 
            }
        } else {
            noImprovementCounter++; 
        }

        // --- SUBIR LA DESTRUCCIÓN SI ESTAMOS ATASCADOS ---
        if (noImprovementCounter >= MAX_NO_IMPROVE) {
            currentDestroySize = min(maxDestroySize, currentDestroySize + 3);
            noImprovementCounter = 0; // Reiniciamos contador para el siguiente salto
        }

        // ── 6. Actualizar pesos adaptativos cada 20 iteraciones ───────────
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
// =============================================================================
// shawRemoval
// Extrae k clientes comenzando por uno aleatorio, luego agrega los más
// cercanos a él usando distancia geográfica como medida de "relación".
// =============================================================================
vector<int> ALNS::shawRemoval(const Solution& sol, int k) {
    const auto& routes = sol.getRoutes();

    // Recolectar todos los clientes presentes en la solución
    vector<int> allClients;
    for (const auto& route : routes) {
        const auto& path = route.getPath();
        for (size_t i = 1; i < path.size() - 1; ++i)
            allClients.push_back(path[i]);
    }

    if (allClients.empty()) return {};
    k = min(k, (int)allClients.size());

    // Elegir cliente semilla al azar
    int seedIdx  = uniform_int_distribution<int>(0, allClients.size()-1)(rng);
    int seedClient = allClients[seedIdx];

    // Ordenar el resto por distancia al cliente semilla
    vector<pair<double,int>> byDistance;
    for (int c : allClients) {
        if (c == seedClient) continue;
        double d = parserData->getDistance(seedClient, c);
        byDistance.push_back({d, c});
    }
    sort(byDistance.begin(), byDistance.end());

    vector<int> removed = {seedClient};
    for (int i = 0; i < k - 1 && i < (int)byDistance.size(); ++i)
        removed.push_back(byDistance[i].second);

    return removed;
}

// =============================================================================
// worstRemoval
// Extrae los k clientes cuya eliminación produce el mayor ahorro en costo,
// es decir, los que están peor insertados en sus rutas actuales.
// =============================================================================
vector<int> ALNS::worstRemoval(const Solution& sol, int k) {
    vector<pair<double,int>> savings;

    for (const auto& route : sol.getRoutes()) {
        const auto& path = route.getPath();
        // path[0] y path.back() son la bodega (ID=1 en base 1)
        for (size_t i = 1; i < path.size() - 1; ++i) {
            int prev   = path[i-1];
            int curr   = path[i];
            int next   = path[i+1];
            double s   = removalSavings(curr, prev, next);
            savings.push_back({s, curr});
        }
    }

    // Ordenar de mayor a menor ahorro
    sort(savings.begin(), savings.end(),
         [](const pair<double,int>& a, const pair<double,int>& b){
             return a.first > b.first;
         });

    k = min(k, (int)savings.size());
    vector<int> removed;
    for (int i = 0; i < k; ++i)
        removed.push_back(savings[i].second);

    return removed;
}

// =============================================================================
// randomRemoval
// Extrae k clientes seleccionados uniformemente al azar.
// =============================================================================
vector<int> ALNS::randomRemoval(const Solution& sol, int k) {
    vector<int> allClients;
    for (const auto& route : sol.getRoutes()) {
        const auto& path = route.getPath();
        for (size_t i = 1; i < path.size() - 1; ++i)
            allClients.push_back(path[i]);
    }

    if (allClients.empty()) return {};
    k = min(k, (int)allClients.size());

    shuffle(allClients.begin(), allClients.end(), rng);
    return vector<int>(allClients.begin(), allClients.begin() + k);
}

// =============================================================================
// insertionCost
// Costo de insertar clientId entre los nodos prev y next.
// Δ = d(prev,client) + d(client,next) - d(prev,next)
// =============================================================================
double ALNS::insertionCost(int clientId, int prev, int next) const {
    return parserData->getDistance(prev, clientId)
         + parserData->getDistance(clientId, next)
         - parserData->getDistance(prev, next);
}

// =============================================================================
// removalSavings
// Ahorro de eliminar clientId que está entre prev y next.
// savings = d(prev,client) + d(client,next) - d(prev,next)
// (mismo valor que insertionCost: es el costo diferencial del arco)
// =============================================================================
double ALNS::removalSavings(int clientId, int prev, int next) const {
    return parserData->getDistance(prev, clientId)
         + parserData->getDistance(clientId, next)
         - parserData->getDistance(prev, next);
}