#include "CbcSolver.h"
#include <iostream>
#include <unistd.h>
#include <coin/OsiClpSolverInterface.hpp>
#include <coin/CbcModel.hpp>
#include <coin/CoinPackedMatrix.hpp>
// Cortes internos de CBC/CGL
#include <coin/CglGomory.hpp>
#include <coin/CglMixedIntegerRounding2.hpp>
#include <coin/CglClique.hpp>
// Heurísticas internas de CBC
#include <coin/CbcHeuristicFPump.hpp>
#include <coin/CbcHeuristicRINS.hpp>
#include <climits>

using namespace std;

CbcSolver::CbcSolver(const Parser* parser) : parserData(parser) {
    numClients = parserData->getDimension();
    // Variables X_ij (N * N) + Variables U_i (N - 1) para MTZ
    numVariables = (numClients * numClients) + (numClients - 1);
}

int CbcSolver::getVarIndex(int i, int j) const {
    return i * numClients + j;
}

int CbcSolver::getUIndex(int i) const {
    return (numClients * numClients) + (i - 1);
}

Solution CbcSolver::convertToSolution(const double* solution) const {
    Solution newSolution(parserData);
    for (int j = 1; j < numClients; ++j) {
        if (solution[getVarIndex(0, j)] > 0.5) {
            Route route(parserData->getCapacity(), parserData);
            int curr = j;
            while (curr != 0) {
                route.addClient(curr + 1);
                int next = -1;
                for (int k = 0; k < numClients; ++k) {
                    if (curr != k && solution[getVarIndex(curr, k)] > 0.5) {
                        next = k;
                        break;
                    }
                }
                curr = next;
            }
            newSolution.addRoute(route);
        }
    }
    return newSolution;
}

Solution CbcSolver::solve(const Solution& warmStart, double timeLimitSeconds) {
    int numVehicles = warmStart.getRoutes().size();
    // =========================================================
    // 1. Interfaz OSI (CLP como solver LP interno de CBC)
    // =========================================================
    OsiClpSolverInterface osi;
    osi.setHintParam(OsiDoReducePrint, true);

    // =========================================================
    // 2. Definición de variables
    // =========================================================
    double* objective = new double[numVariables];
    double* colLower  = new double[numVariables];
    double* colUpper  = new double[numVariables];

    int Q = parserData->getCapacity();

    // Variables X_ij con poda estática de capacidad
    for (int i = 0; i < numClients; ++i) {
        for (int j = 0; j < numClients; ++j) {
            int idx = getVarIndex(i, j);
            objective[idx] = parserData->getDistance(i + 1, j + 1);
            colLower[idx]  = 0.0;

            if (i == j) {
                colUpper[idx] = 0.0;
            } else if (i != 0 && j != 0) {
                int di = parserData->getClients()[i + 1].getDemand();
                int dj = parserData->getClients()[j + 1].getDemand();
                colUpper[idx] = (di + dj > Q) ? 0.0 : 1.0;
            } else {
                colUpper[idx] = 1.0;
            }
        }
    }

    // Variables U_i: carga acumulada (auxiliares MTZ)
    for (int i = 1; i < numClients; ++i) {
        int idx = getUIndex(i);
        objective[idx] = 0.0;
        colLower[idx]  = parserData->getClients()[i + 1].getDemand();
        colUpper[idx]  = Q;
    }

    // =========================================================
    // 3. Construcción de restricciones (CoinPackedMatrix)
    // =========================================================
    CoinPackedMatrix matrix(false, 0, 0);
    matrix.setDimensions(0, numVariables);

    vector<double> rowLowerVec;
    vector<double> rowUpperVec;

    // A. Grado de salida y entrada exactamente 1 por cliente
    for (int i = 1; i < numClients; ++i) {
        vector<int>    indicesOut, indicesIn;
        vector<double> elementsOut, elementsIn;
        for (int j = 0; j < numClients; ++j) {
            if (i != j) {
                indicesOut.push_back(getVarIndex(i, j)); elementsOut.push_back(1.0);
                indicesIn.push_back(getVarIndex(j, i));  elementsIn.push_back(1.0);
            }
        }
        matrix.appendRow(indicesOut.size(), indicesOut.data(), elementsOut.data());
        rowLowerVec.push_back(1.0); rowUpperVec.push_back(1.0);

        matrix.appendRow(indicesIn.size(), indicesIn.data(), elementsIn.data());
        rowLowerVec.push_back(1.0); rowUpperVec.push_back(1.0);
    }

    // B. Restricción de flota: entre 1 y K vehículos
    vector<int>    depotOutIdx, depotInIdx;
    vector<double> depotOutEl,  depotInEl;
    for (int j = 1; j < numClients; ++j) {
        depotOutIdx.push_back(getVarIndex(0, j)); depotOutEl.push_back(1.0);
        depotInIdx.push_back(getVarIndex(j, 0));  depotInEl.push_back(1.0);
    }
    matrix.appendRow(depotOutIdx.size(), depotOutIdx.data(), depotOutEl.data());
    rowLowerVec.push_back(1.0); rowUpperVec.push_back((double)numVehicles);

    matrix.appendRow(depotInIdx.size(), depotInIdx.data(), depotInEl.data());
    rowLowerVec.push_back(1.0); rowUpperVec.push_back((double)numVehicles);

    // C. Lifted MTZ (Desrochers & Laporte)
    //    U_i - U_j + Q*X_ij + (Q - d_i - d_j)*X_ji <= Q - d_j
    for (int i = 1; i < numClients; ++i) {
        for (int j = 1; j < numClients; ++j) {
            if (i != j) {
                int di = parserData->getClients()[i + 1].getDemand();
                int dj = parserData->getClients()[j + 1].getDemand();

                vector<int>    indices  = {getUIndex(i), getUIndex(j), getVarIndex(i, j), getVarIndex(j, i)};
                vector<double> elements = {1.0, -1.0, (double)Q, (double)(Q - di - dj)};
                double rhs = Q - dj;

                matrix.appendRow(indices.size(), indices.data(), elements.data());
                rowLowerVec.push_back(-COIN_DBL_MAX); rowUpperVec.push_back(rhs);
            }
        }
    }

    // D. Prohibición de 2-ciclos: X_ij + X_ji <= 1
    for (int i = 1; i < numClients; ++i) {
        for (int j = i + 1; j < numClients; ++j) {
            vector<int>    indices  = {getVarIndex(i, j), getVarIndex(j, i)};
            vector<double> elements = {1.0, 1.0};
            matrix.appendRow(indices.size(), indices.data(), elements.data());
            rowLowerVec.push_back(-COIN_DBL_MAX); rowUpperVec.push_back(1.0);
        }
    }

    osi.loadProblem(matrix, colLower, colUpper, objective,
                    rowLowerVec.data(), rowUpperVec.data());

    // =========================================================
    // 4. Declarar integralidad de variables X_ij
    // =========================================================
    for (int i = 0; i < numClients; ++i)
        for (int j = 0; j < numClients; ++j)
            osi.setInteger(getVarIndex(i, j));

    // =========================================================
    // 5. Crear el modelo CBC
    // =========================================================
    CbcModel model(osi);
    model.setLogLevel(0);

    // =========================================================
    // 6. Warm Start: Clarke-Wright + VNS
    // =========================================================
    VNS vns(parserData);

    double* mipStart = new double[numVariables];
    std::fill(mipStart, mipStart + numVariables, 0.0);

    for (const auto& route : warmStart.getRoutes()) {
        const auto& path = route.getPath();
        for (size_t i = 0; i < path.size() - 1; ++i)
            mipStart[getVarIndex(path[i] - 1, path[i + 1] - 1)] = 1.0;
    }

    for (const auto& route : warmStart.getRoutes()) {
        const auto& path = route.getPath();
        int accumulatedLoad = 0;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            int clientId = path[i];
            int lpIndex  = clientId - 1;
            accumulatedLoad += parserData->getClients()[clientId].getDemand();
            if (lpIndex >= 1 && lpIndex < numClients)
                mipStart[getUIndex(lpIndex)] = accumulatedLoad;
        }
    }

    model.setBestSolution(mipStart, numVariables, warmStart.getTotalCost());
    model.setCutoff(warmStart.getTotalCost() + 0.999);

    // =========================================================
    // 7. Prioridades de ramificación
    // =========================================================
    int* priorities = new int[numVariables];
    for (int i = 0; i < numVariables; i++) priorities[i] = 1000;
    for (int i = 0; i < numClients; ++i)
        for (int j = 0; j < numClients; ++j)
            priorities[getVarIndex(i, j)] = 1;
    model.passInPriorities(priorities, false);

    // =========================================================
    // 8. Cortes DFJ lazy (SubtourCutGenerator)
    // =========================================================
    SubtourCutGenerator subtourGen(parserData, numClients);
    model.addCutGenerator(&subtourGen, 1, "SubtourDFJ");

    // =========================================================
    // 9. Cortes internos de CBC/CGL
    // =========================================================
    CglGomory gomory;
    gomory.setLimit(100);
    model.addCutGenerator(&gomory, 1, "Gomory");

    CglMixedIntegerRounding2 mir;
    model.addCutGenerator(&mir, 1, "MIR");

    CglClique clique;
    model.addCutGenerator(&clique, 1, "Clique");

    // =========================================================
    // 10. Heurísticas internas de CBC
    // =========================================================
    CbcHeuristicFPump fpump(model);
    fpump.setMaximumPasses(30);
    model.addHeuristic(&fpump, "FeasibilityPump");

    CbcHeuristicRINS rins(model);
    rins.setWhen(10);
    model.addHeuristic(&rins, "RINS");

    // =========================================================
    // 11. Límite de tiempo
    // =========================================================
    model.setMaximumSeconds(timeLimitSeconds);

    // =========================================================
    // 12. Resolver
    // =========================================================

    // Handler: reportes compactos + VNS sobre cada solución entera encontrada
    CbcProgressHandler progressHandler(parserData, &vns);
    model.passInEventHandler(&progressHandler);

    cout << "\n=== CBC | " << numClients << " clientes"
         << " | Warm start: " << warmStart.getTotalCost()
         << " | Limite: " << timeLimitSeconds << "s ===" << endl;

    // Suprimir stdout interno de CBC/CGL (clique cuts, gomory, etc.)
    int savedStdout = dup(STDOUT_FILENO);
    freopen("/dev/null", "w", stdout);

    model.branchAndBound();

    // Restaurar stdout para que los prints posteriores sean visibles
    fflush(stdout);
    dup2(savedStdout, STDOUT_FILENO);
    close(savedStdout);
    std::cout.clear();

    // =========================================================
    // 13. Extraer y refinar la mejor solución encontrada
    // =========================================================
    Solution bestSolution(parserData);

    bool hasRealSolution = (model.bestSolution() != nullptr)
                        && (model.getObjValue() < 1e+49);

    if (model.isProvenOptimal()) {
        cout << "\nCBC encontro solucion OPTIMA. Costo: " << model.getObjValue() << endl;
        bestSolution = convertToSolution(model.bestSolution());
    } else if (hasRealSolution) {
        cout << "\nCBC finalizo por limite de tiempo." << endl;
        cout << "Mejor costo MIP encontrado: " << model.getObjValue() << endl;
        cout << "Mejor cota inferior (LB):   " << model.getBestPossibleObjValue() << endl;
        cout << "GAP: " << 100.0 * (model.getObjValue() - model.getBestPossibleObjValue())
                           / model.getObjValue() << "%" << endl;
        bestSolution = convertToSolution(model.bestSolution());
    } else {
        cout << "\nCBC no encontro solucion entera (problema infactible o sin solucion)." << endl;
        bestSolution = warmStart;
    }

    // Refinamiento final con VNS
    double costBeforeVNS = bestSolution.getTotalCost();
    bestSolution = vns.optimize(bestSolution);
    cout << "Costo tras refinamiento VNS final: " << bestSolution.getTotalCost();
    if (bestSolution.getTotalCost() < costBeforeVNS)
        cout << "  (mejora de " << costBeforeVNS - bestSolution.getTotalCost() << ")";
    cout << endl;

    // =========================================================
    // 14. Limpieza de memoria
    // =========================================================
    delete[] objective;
    delete[] colLower;
    delete[] colUpper;
    delete[] mipStart;
    delete[] priorities;

    return bestSolution;
}

// solveSubproblem
// Núcleo del LNS-MIP: recibe la solución actual y los clientes liberados
// por el operador destroy de ALNS, construye un mini-CVRP solo con esos
// clientes, lo resuelve óptimamente con CBC y reconstruye la solución
// completa fusionando las mini-rutas con los stubs de las rutas afectadas.
// =============================================================================
Solution CbcSolver::solveSubproblem(const Solution&         currentSol,
                                    const vector<int>&      freeClients,
                                    double                  timeLimitSeconds) {
    if (freeClients.empty()) return currentSol;

    int Q = parserData->getCapacity();
    set<int> freeSet(freeClients.begin(), freeClients.end());
    int k = (int)freeClients.size();

    // ── 1. Separar rutas fijas y afectadas ───────────────────────────────
    // Rutas fijas: ninguno de sus clientes está en freeSet → se conservan tal cual.
    // Rutas afectadas: contienen al menos un cliente libre → se abren.
    //   stub = clientes NO libres que quedan en la ruta afectada.
    vector<Route>        fixedRoutes;
    vector<vector<int>>  stubs;      // secuencia de clientes fijos por ruta afectada
    vector<int>          stubLoads;  // demanda acumulada de cada stub

    for (const auto& route : currentSol.getRoutes()) {
        const auto& path = route.getPath();
        bool affected = false;
        for (int id : path)
            if (freeSet.count(id)) { affected = true; break; }

        if (!affected) {
            fixedRoutes.push_back(route);
        } else {
            vector<int> stub;
            int load = 0;
            // path[0] y path.back() son la bodega (ID=1)
            for (size_t p = 1; p < path.size() - 1; ++p) {
                int id = path[p];
                if (!freeSet.count(id)) {
                    stub.push_back(id);
                    load += parserData->getClients()[id].getDemand();
                }
            }
            stubs.push_back(stub);
            stubLoads.push_back(load);
        }
    }

    int numAffected = (int)stubs.size();

    // ── 2. Construir mini-CVRP ────────────────────────────────────────────
    // Nodos del mini-modelo:
    //   LP índice 0       → depot  (real ID = 1)
    //   LP índice 1..k    → freeClients[0..k-1]
    //
    // Variables: X_ij (miniN * miniN) + U_i (k variables MTZ)

    int miniN        = k + 1;
    int numXVars     = miniN * miniN;
    int totalMiniVar = numXVars + k;   // +k para U_1..U_k

    // Conversión nodo LP ↔ real ID
    auto nodeToRealId = [&](int i) -> int {
        return (i == 0) ? 1 : freeClients[i - 1];
    };
    auto xIdx = [&](int i, int j) { return i * miniN + j; };
    auto uIdx = [&](int i) { return numXVars + (i - 1); }; // i en 1..k

    double* mObj = new double[totalMiniVar]();
    double* mLb  = new double[totalMiniVar]();
    double* mUb  = new double[totalMiniVar]();

    // Variables X_ij
    for (int i = 0; i < miniN; ++i) {
        for (int j = 0; j < miniN; ++j) {
            int idx = xIdx(i, j);
            mObj[idx] = parserData->getDistance(nodeToRealId(i), nodeToRealId(j));
            mLb[idx]  = 0.0;
            mUb[idx]  = (i == j) ? 0.0 : 1.0;
        }
    }

    // Variables U_i (continuas, rango [demanda_i, Q])
    for (int i = 1; i <= k; ++i) {
        int realId    = nodeToRealId(i);
        mObj[uIdx(i)] = 0.0;
        mLb[uIdx(i)]  = parserData->getClients()[realId].getDemand();
        mUb[uIdx(i)]  = Q;
    }

    // ── 3. Restricciones del mini-modelo ─────────────────────────────────
    CoinPackedMatrix miniMat(false, 0, 0);
    miniMat.setDimensions(0, totalMiniVar);
    vector<double> rLo, rHi;

    // A. Grado de salida y entrada = 1 por cada cliente libre
    for (int i = 1; i < miniN; ++i) {
        vector<int> oI, iI; vector<double> oE, iE;
        for (int j = 0; j < miniN; ++j) {
            if (i == j) continue;
            oI.push_back(xIdx(i,j)); oE.push_back(1.0);
            iI.push_back(xIdx(j,i)); iE.push_back(1.0);
        }
        miniMat.appendRow(oI.size(), oI.data(), oE.data());
        rLo.push_back(1.0); rHi.push_back(1.0);
        miniMat.appendRow(iI.size(), iI.data(), iE.data());
        rLo.push_back(1.0); rHi.push_back(1.0);
    }

    // B. Flota: entre 1 y (numAffected+1) rutas desde depot
    //    +1 permite crear una ruta nueva si ningún stub tiene capacidad suficiente
    int maxMiniVehicles = numAffected + 1;
    {
        vector<int> dO, dI; vector<double> dOe, dIe;
        for (int j = 1; j < miniN; ++j) {
            dO.push_back(xIdx(0,j)); dOe.push_back(1.0);
            dI.push_back(xIdx(j,0)); dIe.push_back(1.0);
        }
        miniMat.appendRow(dO.size(), dO.data(), dOe.data());
        rLo.push_back(1.0); rHi.push_back((double)maxMiniVehicles);
        miniMat.appendRow(dI.size(), dI.data(), dIe.data());
        rLo.push_back(1.0); rHi.push_back((double)maxMiniVehicles);
    }

    // C. Lifted MTZ: U_i - U_j + Q*X_ij + (Q-di-dj)*X_ji <= Q-dj
    for (int i = 1; i <= k; ++i) {
        for (int j = 1; j <= k; ++j) {
            if (i == j) continue;
            int di = parserData->getClients()[nodeToRealId(i)].getDemand();
            int dj = parserData->getClients()[nodeToRealId(j)].getDemand();
            vector<int>    idx = {uIdx(i), uIdx(j), xIdx(i,j), xIdx(j,i)};
            vector<double> el  = {1.0, -1.0, (double)Q, (double)(Q - di - dj)};
            miniMat.appendRow(idx.size(), idx.data(), el.data());
            rLo.push_back(-COIN_DBL_MAX);
            rHi.push_back((double)(Q - dj));
        }
    }

    // D. Prohibición de 2-ciclos: X_ij + X_ji <= 1
    for (int i = 1; i <= k; ++i) {
        for (int j = i+1; j <= k; ++j) {
            vector<int>    idx = {xIdx(i,j), xIdx(j,i)};
            vector<double> el  = {1.0, 1.0};
            miniMat.appendRow(idx.size(), idx.data(), el.data());
            rLo.push_back(-COIN_DBL_MAX);
            rHi.push_back(1.0);
        }
    }

    // ── 4. Armar y resolver modelo CBC ───────────────────────────────────
    OsiClpSolverInterface miniOsi;
    miniOsi.setHintParam(OsiDoReducePrint, true);
    miniOsi.loadProblem(miniMat, mLb, mUb, mObj, rLo.data(), rHi.data());

    for (int i = 0; i < miniN; ++i)
        for (int j = 0; j < miniN; ++j)
            miniOsi.setInteger(xIdx(i, j));

    CbcModel miniModel(miniOsi);
    miniModel.setLogLevel(0);
    miniModel.setMaximumSeconds(timeLimitSeconds);

    // Suprimir stdout de CBC durante el subproblema
    int savedFd = dup(STDOUT_FILENO);
    freopen("/dev/null", "w", stdout);

    miniModel.branchAndBound();

    fflush(stdout);
    dup2(savedFd, STDOUT_FILENO);
    close(savedFd);
    std::cout.clear();

    // ── 5. Reconstruir si CBC encontró solución ───────────────────────────
    if (!miniModel.bestSolution() || miniModel.getObjValue() >= 1e49) {
        delete[] mObj; delete[] mLb; delete[] mUb;
        return currentSol; // fallback: devolver solución sin cambios
    }

    const double* miniSol = miniModel.bestSolution();

    // Extraer mini-rutas en real IDs
    vector<vector<int>> miniRoutes;
    for (int j = 1; j < miniN; ++j) {
        if (miniSol[xIdx(0, j)] <= 0.5) continue;
        vector<int> mRoute;
        int curr = j;
        while (curr != 0) {
            mRoute.push_back(nodeToRealId(curr));
            int next = -1;
            for (int l = 0; l < miniN; ++l)
                if (curr != l && miniSol[xIdx(curr, l)] > 0.5) { next = l; break; }
            curr = next;
            if (curr == -1) break;
        }
        if (!mRoute.empty()) miniRoutes.push_back(mRoute);
    }

    delete[] mObj; delete[] mLb; delete[] mUb;

    return mergeSubproblemResult(fixedRoutes, stubs, stubLoads, miniRoutes);
}

// =============================================================================
// mergeSubproblemResult
// Estrategia de fusión Greedy-Distance (Inserción Espacial):
// Evalúa insertar la mini-ruta completa en todas las posiciones posibles de 
// todos los stubs con capacidad suficiente, eligiendo la que minimice el 
// salto geográfico (Delta Z).
// =============================================================================
Solution CbcSolver::mergeSubproblemResult(
    const vector<Route>&          fixedRoutes,
    const vector<vector<int>>&    stubsOriginal,
    const vector<int>&            stubLoadsOriginal,
    const vector<vector<int>>&    miniRoutes) const {

    int Q = parserData->getCapacity();
    Solution result(parserData);

    // Copiar rutas fijas sin cambios
    for (const auto& r : fixedRoutes)
        result.addRoute(r);

    // Hacemos copias mutables de los stubs para poder insertar dentro de ellos
    vector<vector<int>> stubs = stubsOriginal;
    vector<int> stubLoads = stubLoadsOriginal;

    auto miniDemand = [&](const vector<int>& mr) {
        int d = 0;
        for (int id : mr) d += parserData->getClients()[id].getDemand();
        return d;
    };

    // Fusión por distancia
    for (const auto& mr : miniRoutes) {
        int md = miniDemand(mr);
        int first_mr = mr.front();
        int last_mr = mr.back();

        int bestSi = -1;
        int bestPos = -1;
        int bestDelta = INT_MAX;

        for (int si = 0; si < (int)stubs.size(); ++si) {
            if (stubLoads[si] + md > Q) continue; // No cabe por capacidad

            // Probar todas las posiciones de inserción dentro de este stub
            int stubSize = stubs[si].size();
            for (int pos = 0; pos <= stubSize; ++pos) {
                int prev = (pos == 0) ? 1 : stubs[si][pos - 1]; // 1 es la bodega
                int next = (pos == stubSize) ? 1 : stubs[si][pos];

                // Costo de romper el enlace prev->next e insertar mr completo
                int delta = parserData->getDistance(prev, first_mr) 
                          + parserData->getDistance(last_mr, next) 
                          - parserData->getDistance(prev, next);

                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestSi = si;
                    bestPos = pos;
                }
            }
        }

        if (bestSi >= 0) {
            // Inserción óptima encontrada: metemos mr dentro del stub elegido
            stubs[bestSi].insert(stubs[bestSi].begin() + bestPos, mr.begin(), mr.end());
            stubLoads[bestSi] += md;
        } else {
            // No cabe en ningún stub existente, se convierte en un nuevo stub (nueva ruta)
            stubs.push_back(mr);
            stubLoads.push_back(md);
        }
    }

    // Transformar todos los stubs modificados en objetos Route formales
    for (int si = 0; si < (int)stubs.size(); ++si) {
        if (stubs[si].empty()) continue;
        Route newRoute(Q, parserData);
        for (int id : stubs[si]) {
            newRoute.addClient(id);
        }
        result.addRoute(newRoute);
    }

    return result;
}