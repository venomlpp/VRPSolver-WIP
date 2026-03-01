#include "CbcSolver.h"
#include <iostream>
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

Solution CbcSolver::solve(int numVehicles, double timeLimitSeconds) {
    // =========================================================
    // 1. Interfaz OSI (CLP como solver LP interno de CBC)
    // =========================================================
    OsiClpSolverInterface osi;
    osi.setHintParam(OsiDoReducePrint, true); // Silenciar output de CLP

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
                colUpper[idx] = 0.0; // Sin autoloops
            } else if (i != 0 && j != 0) {
                // Poda estática: si dos clientes juntos explotan el camión
                // nunca pueden ser adyacentes en ninguna solución factible
                int di = parserData->getClients()[i + 1].getDemand();
                int dj = parserData->getClients()[j + 1].getDemand();
                colUpper[idx] = (di + dj > Q) ? 0.0 : 1.0;
            } else {
                colUpper[idx] = 1.0;
            }
        }
    }

    // Variables U_i: carga acumulada del vehículo al llegar al cliente i
    // (auxiliares para MTZ, no forman parte de la función objetivo)
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

    // B. Restricción de flota: entre 1 y K vehículos salen/entran a la bodega.
    //    MEJORA: usar <= K en vez de == K permite que CBC explore soluciones
    //    con menos vehículos (a veces producen rutas más cortas).
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

    // C. Lifted MTZ (Desrochers & Laporte): más fuerte que MTZ estándar.
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

    // Cargar el modelo completo en OSI
    osi.loadProblem(matrix, colLower, colUpper, objective,
                    rowLowerVec.data(), rowUpperVec.data());

    // =========================================================
    // 4. Declarar integralidad de variables X_ij
    // =========================================================
    for (int i = 0; i < numClients; ++i)
        for (int j = 0; j < numClients; ++j)
            osi.setInteger(getVarIndex(i, j));
    // Nota: U_i son continuas, no se declaran enteras (MTZ solo necesita eso)

    // =========================================================
    // 5. Crear el modelo CBC
    // =========================================================
    CbcModel model(osi);
    model.setLogLevel(0);

    // =========================================================
    // 6. Warm Start: Clarke-Wright + VNS
    //    Le entregamos a CBC una solución inicial de alta calidad.
    //    Esto establece una cota superior (UB) ajustada desde el
    //    primer nodo, lo que permite podar ramas mucho antes.
    // =========================================================
    GreedyBuilder builder(parserData);
    Solution initialSol = builder.buildSolution();

    VNS vns(parserData);
    Solution refinedInitialSol = vns.optimize(initialSol, 30);

    double* mipStart = new double[numVariables];
    std::fill(mipStart, mipStart + numVariables, 0.0);

    // Mapear las rutas VNS a variables binarias X_ij = 1.0
    for (const auto& route : refinedInitialSol.getRoutes()) {
        const auto& path = route.getPath();
        for (size_t i = 0; i < path.size() - 1; ++i)
            mipStart[getVarIndex(path[i] - 1, path[i + 1] - 1)] = 1.0;
    }

    // También rellenar variables U_i en el warm start con valores factibles
    // (mejora la aceptación del MIP start por parte de CBC)
    for (const auto& route : refinedInitialSol.getRoutes()) {
        const auto& path = route.getPath();
        int accumulatedLoad = 0;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            int clientId = path[i];   // ID con base 1 (path usa IDs reales)
            int lpIndex  = clientId - 1; // Índice LP base 0
            accumulatedLoad += parserData->getClients()[clientId].getDemand();
            if (lpIndex >= 1 && lpIndex < numClients)
                mipStart[getUIndex(lpIndex)] = accumulatedLoad;
        }
    }

    model.setBestSolution(mipStart, numVariables, refinedInitialSol.getTotalCost());

    // MEJORA: +0.999 evita que CBC poda la solución óptima por error de
    // punto flotante cuando los costos son enteros
    model.setCutoff(refinedInitialSol.getTotalCost() + 0.999);

    // =========================================================
    // 7. Prioridades de ramificación
    //    X_ij primero (definen las rutas), U_i después (auxiliares MTZ)
    // =========================================================
    int* priorities = new int[numVariables];
    for (int i = 0; i < numVariables; i++) priorities[i] = 1000;
    for (int i = 0; i < numClients; ++i)
        for (int j = 0; j < numClients; ++j)
            priorities[getVarIndex(i, j)] = 1;
    model.passInPriorities(priorities, false);

    // =========================================================
    // 8. Cortes DFJ lazy (SubtourCutGenerator)
    //    Detecta subtours y violaciones de capacidad en cada nodo
    //    e inyecta cortes DFJ. Complementa el MTZ levantando el LB
    //    mucho más rápido que MTZ solo.
    // =========================================================
    SubtourCutGenerator subtourGen(parserData, numClients);
    model.addCutGenerator(&subtourGen, 1, "SubtourDFJ");

    // =========================================================
    // 9. Cortes internos de CBC/CGL
    // =========================================================

    // Gomory cuts: derivan de la base simplex, muy efectivos para MILP
    CglGomory gomory;
    gomory.setLimit(100);
    model.addCutGenerator(&gomory, 1, "Gomory");

    // Mixed Integer Rounding: complementa a Gomory en variables mixtas
    CglMixedIntegerRounding2 mir;
    model.addCutGenerator(&mir, 1, "MIR");

    // Clique cuts: útiles dado que tenemos restricciones de 2-ciclos
    CglClique clique;
    model.addCutGenerator(&clique, 1, "Clique");

    // =========================================================
    // 10. Heurísticas internas de CBC
    //     Buscan soluciones enteras factibles sin explorar el árbol completo
    // =========================================================

    // Feasibility Pump: proyecta la solución LP hacia factibilidad entera
    CbcHeuristicFPump fpump(model);
    fpump.setMaximumPasses(30);
    model.addHeuristic(&fpump, "FeasibilityPump");

    // RINS: fija variables que coinciden entre la solución LP y la mejor
    // solución entera conocida, y resuelve el subproblema reducido
    CbcHeuristicRINS rins(model);
    rins.setWhen(10); // Ejecutar cada 10 nodos
    model.addHeuristic(&rins, "RINS");

    // =========================================================
    // 11. Límite de tiempo (reemplaza límite de nodos)
    //     Con MTZ+DFJ el árbol puede ser muy profundo; el tiempo
    //     es un límite más robusto que el número de nodos.
    // =========================================================
    model.setMaximumSeconds(timeLimitSeconds); // tiempo ajustable arriba

    // =========================================================
    // 12. Resolver
    // =========================================================
    cout << "\n=== INICIANDO CBC BRANCH & CUT (MTZ + DFJ + VNS) ===" << endl;
    cout << "Costo VNS inicial inyectado como warm start: "
         << refinedInitialSol.getTotalCost() << endl;

    model.branchAndBound();

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
        bestSolution = refinedInitialSol; // Retorna warm start como fallback
    }

    // Refinamiento final con VNS: pule la solución de CBC
    // (puede mejorar si CBC dejó alguna ruta subóptima por el límite de tiempo)
    double costBeforeVNS = bestSolution.getTotalCost();
    bestSolution = vns.optimize(bestSolution, 30);
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