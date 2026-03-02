#include "CbcSolver.h"
#include <iostream>
#include <unistd.h>
#include <coin/OsiClpSolverInterface.hpp>
#include <coin/CbcModel.hpp>
#include <coin/CoinPackedMatrix.hpp>
#include <coin/CglGomory.hpp>
#include <coin/CglMixedIntegerRounding2.hpp>
#include <coin/CglClique.hpp>
#include <coin/CbcHeuristicFPump.hpp>
#include <coin/CbcHeuristicRINS.hpp>
#include <climits>

using namespace std;

/*
 * Descripción: Constructor del solver exacto.
 * Entrada: Puntero a la instancia del parser.
 * Salida: Instancia inicializada configurando la cantidad de variables X_ij y U_i.
 */
CbcSolver::CbcSolver(const Parser* parser) : parserData(parser) {
    numClients = parserData->getDimension();
    numVariables = (numClients * numClients) + (numClients - 1);
}

/*
 * Descripción: Mapea coordenadas 2D a un índice lineal 1D para variables binarias.
 * Entrada: Índices de nodo i, j.
 * Salida: Índice de la variable X_ij en el modelo LP.
 */
int CbcSolver::getVarIndex(int i, int j) const {
    return i * numClients + j;
}

/*
 * Descripción: Mapea el índice del cliente a su variable continua U_i para la formulación MTZ.
 * Entrada: Índice del cliente.
 * Salida: Índice de la variable U_i en el modelo LP.
 */
int CbcSolver::getUIndex(int i) const {
    return (numClients * numClients) + (i - 1);
}

/*
 * Descripción: Convierte el vector LP resultante en un objeto Solution estructurado.
 * Entrada: Arreglo de variables evaluadas por el solver.
 * Salida: Objeto Solution validado.
 */
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

/*
 * Descripción: Ejecuta la resolución Branch & Cut del CVRP completo usando la librería CBC.
 * Entrada: Solución heurística inicial (Warm Start) y límite de tiempo en segundos.
 * Salida: Mejor solución entera encontrada.
 */
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
    // 6. Warm Start
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
    // 8. Cortes y Heurísticas internas de CBC/CGL
    // =========================================================
    SubtourCutGenerator subtourGen(parserData, numClients);
    model.addCutGenerator(&subtourGen, 1, "SubtourDFJ");

    CglGomory gomory;
    gomory.setLimit(100);
    model.addCutGenerator(&gomory, 1, "Gomory");

    CglMixedIntegerRounding2 mir;
    model.addCutGenerator(&mir, 1, "MIR");

    CglClique clique;
    model.addCutGenerator(&clique, 1, "Clique");

    CbcHeuristicFPump fpump(model);
    fpump.setMaximumPasses(30);
    model.addHeuristic(&fpump, "FeasibilityPump");

    CbcHeuristicRINS rins(model);
    rins.setWhen(10);
    model.addHeuristic(&rins, "RINS");

    model.setMaximumSeconds(timeLimitSeconds);

    // =========================================================
    // 9. Resolver
    // =========================================================
    CbcProgressHandler progressHandler(parserData, &vns);
    model.passInEventHandler(&progressHandler);

    cout << "\n=== CBC | " << numClients << " clientes"
         << " | Warm start: " << warmStart.getTotalCost()
         << " | Limite: " << timeLimitSeconds << "s ===" << endl;

    // Suprimir stdout interno de CBC/CGL para mantener limpia la consola
    int savedStdout = dup(STDOUT_FILENO);
    freopen("/dev/null", "w", stdout);

    model.branchAndBound();

    fflush(stdout);
    dup2(savedStdout, STDOUT_FILENO);
    close(savedStdout);
    std::cout.clear();

    // =========================================================
    // 10. Extraer y refinar la mejor solución encontrada
    // =========================================================
    Solution bestSolution(parserData);
    bool hasRealSolution = (model.bestSolution() != nullptr) && (model.getObjValue() < 1e+49);

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

    double costBeforeVNS = bestSolution.getTotalCost();
    bestSolution = vns.optimize(bestSolution);
    cout << "Costo tras refinamiento VNS final: " << bestSolution.getTotalCost();
    if (bestSolution.getTotalCost() < costBeforeVNS)
        cout << "  (mejora de " << costBeforeVNS - bestSolution.getTotalCost() << ")";
    cout << endl;

    // Limpieza de memoria
    delete[] objective;
    delete[] colLower;
    delete[] colUpper;
    delete[] mipStart;
    delete[] priorities;

    return bestSolution;
}

/*
 * Descripción: Núcleo del LNS-MIP. Recibe la solución actual y los clientes liberados
 * por el operador destroy de ALNS, construye un mini-CVRP y lo resuelve óptimamente.
 * Entrada: Solución base actual, vector de clientes extraídos, tiempo límite.
 * Salida: Solución completa fusionando stubs intactos con nuevas mini-rutas.
 */
Solution CbcSolver::solveSubproblem(const Solution&         currentSol,
                                    const vector<int>&      freeClients,
                                    double                  timeLimitSeconds) {
    if (freeClients.empty()) return currentSol;

    int Q = parserData->getCapacity();
    set<int> freeSet(freeClients.begin(), freeClients.end());
    int k = (int)freeClients.size();

    // ── 1. Separar rutas fijas y afectadas ───────────────────────────────
    vector<Route>        fixedRoutes;
    vector<vector<int>>  stubs;      
    vector<int>          stubLoads;  

    for (const auto& route : currentSol.getRoutes()) {
        const auto& path = route.getPath();
        bool affected = false;
        for (int id : path) {
            if (freeSet.count(id)) { affected = true; break; }
        }

        if (!affected) {
            fixedRoutes.push_back(route);
        } else {
            vector<int> stub;
            int load = 0;
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
    int miniN        = k + 1;
    int numXVars     = miniN * miniN;
    int totalMiniVar = numXVars + k;

    auto nodeToRealId = [&](int i) -> int {
        return (i == 0) ? 1 : freeClients[i - 1];
    };
    auto xIdx = [&](int i, int j) { return i * miniN + j; };
    auto uIdx = [&](int i) { return numXVars + (i - 1); };

    double* mObj = new double[totalMiniVar]();
    double* mLb  = new double[totalMiniVar]();
    double* mUb  = new double[totalMiniVar]();

    for (int i = 0; i < miniN; ++i) {
        for (int j = 0; j < miniN; ++j) {
            int idx = xIdx(i, j);
            mObj[idx] = parserData->getDistance(nodeToRealId(i), nodeToRealId(j));
            mLb[idx]  = 0.0;
            mUb[idx]  = (i == j) ? 0.0 : 1.0;
        }
    }

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

    // A. Grado de salida y entrada
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

    // B. Flota
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

    // C. Lifted MTZ
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

    // D. Prohibición de 2-ciclos
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

    for (int i = 0; i < miniN; ++i) {
        for (int j = 0; j < miniN; ++j) {
            miniOsi.setInteger(xIdx(i, j));
        }
    }

    CbcModel miniModel(miniOsi);
    miniModel.setLogLevel(0);
    miniModel.setMaximumSeconds(timeLimitSeconds);

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
        return currentSol; 
    }

    const double* miniSol = miniModel.bestSolution();

    vector<vector<int>> miniRoutes;
    for (int j = 1; j < miniN; ++j) {
        if (miniSol[xIdx(0, j)] <= 0.5) continue;
        vector<int> mRoute;
        int curr = j;
        while (curr != 0) {
            mRoute.push_back(nodeToRealId(curr));
            int next = -1;
            for (int l = 0; l < miniN; ++l) {
                if (curr != l && miniSol[xIdx(curr, l)] > 0.5) { 
                    next = l; 
                    break; 
                }
            }
            curr = next;
            if (curr == -1) break;
        }
        if (!mRoute.empty()) miniRoutes.push_back(mRoute);
    }

    delete[] mObj; delete[] mLb; delete[] mUb;

    return mergeSubproblemResult(fixedRoutes, stubs, stubLoads, miniRoutes);
}

/*
 * Descripción: Estrategia de fusión espacial (Greedy-Distance). Inserta las 
 * mini-rutas calculadas por CBC en la posición geométrica más económica dentro de los stubs.
 * Entrada: Rutas completas, stubs rotos, demandas parciales, mini-rutas óptimas.
 * Salida: Solución validada.
 */
Solution CbcSolver::mergeSubproblemResult(
    const vector<Route>&          fixedRoutes,
    const vector<vector<int>>&    stubsOriginal,
    const vector<int>&            stubLoadsOriginal,
    const vector<vector<int>>&    miniRoutes) const {

    int Q = parserData->getCapacity();
    Solution result(parserData);

    for (const auto& r : fixedRoutes) {
        result.addRoute(r);
    }

    vector<vector<int>> stubs = stubsOriginal;
    vector<int> stubLoads = stubLoadsOriginal;

    auto miniDemand = [&](const vector<int>& mr) {
        int d = 0;
        for (int id : mr) d += parserData->getClients()[id].getDemand();
        return d;
    };

    vector<vector<int>> sortedMiniRoutes = miniRoutes;
    sort(sortedMiniRoutes.begin(), sortedMiniRoutes.end(),
         [&](const vector<int>& a, const vector<int>& b) {
             return miniDemand(a) > miniDemand(b);
         });

    for (const auto& mr : sortedMiniRoutes) {
        int md = miniDemand(mr);
        int first_mr = mr.front();
        int last_mr  = mr.back();

        int bestSi    = -1;
        int bestPos   = -1;
        int bestDelta = INT_MAX;

        for (int si = 0; si < (int)stubs.size(); ++si) {
            if (stubLoads[si] + md > Q) continue;

            int stubSize = stubs[si].size();
            for (int pos = 0; pos <= stubSize; ++pos) {
                int prev = (pos == 0)        ? 1 : stubs[si][pos - 1];
                int next = (pos == stubSize) ? 1 : stubs[si][pos];

                int delta = parserData->getDistance(prev, first_mr)
                          + parserData->getDistance(last_mr, next)
                          - parserData->getDistance(prev, next);

                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestSi    = si;
                    bestPos   = pos;
                }
            }
        }

        if (bestSi >= 0) {
            stubs[bestSi].insert(stubs[bestSi].begin() + bestPos,
                                 mr.begin(), mr.end());
            stubLoads[bestSi] += md;
        } else {
            stubs.push_back(mr);
            stubLoads.push_back(md);
        }
    }

    for (int si = 0; si < (int)stubs.size(); ++si) {
        if (stubs[si].empty()) continue;
        Route newRoute(Q, parserData);
        for (int id : stubs[si]) newRoute.addClient(id);
        result.addRoute(newRoute);
    }

    return result;
}