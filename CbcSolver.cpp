#include "CbcSolver.h"
#include <iostream>
#include <coin/OsiClpSolverInterface.hpp>
#include <coin/CbcModel.hpp>
#include <coin/CoinPackedMatrix.hpp>
#include "GreedyBuilder.h" // NUEVO: Para el Warm Start

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

Solution CbcSolver::solve(int numVehicles) {
    // 1. Instanciamos la Interfaz OSI
    OsiClpSolverInterface osi;
    
    // 2. Definimos arreglos de variables
    double* objective = new double[numVariables];
    double* colLower = new double[numVariables];
    double* colUpper = new double[numVariables];
    
    int Q = parserData->getCapacity();

    // Variables X_ij + MEJORA 1: Pre-procesamiento de Capacidad
    for (int i = 0; i < numClients; ++i) {
        for (int j = 0; j < numClients; ++j) {
            int idx = getVarIndex(i, j);
            objective[idx] = parserData->getDistance(i + 1, j + 1); 
            colLower[idx] = 0.0; 
            
            if (i == j) {
                colUpper[idx] = 0.0;
            } else if (i != 0 && j != 0) { 
                // Si ambos son clientes, revisamos si juntos explotan el camión
                int di = parserData->getClients()[i + 1].getDemand();
                int dj = parserData->getClients()[j + 1].getDemand();
                if (di + dj > Q) {
                    colUpper[idx] = 0.0; // Poda estática absoluta
                } else {
                    colUpper[idx] = 1.0;
                }
            } else {
                colUpper[idx] = 1.0;
            }
        }
    }
    
    // Variables U_i (Cargas continuas para MTZ)
    for (int i = 1; i < numClients; ++i) {
        int idx = getUIndex(i);
        objective[idx] = 0.0; 
        colLower[idx] = parserData->getClients()[i + 1].getDemand(); 
        colUpper[idx] = Q;
    }

    // 3. CONSTRUCCIÓN UNIVERSAL (CoinPackedMatrix)
    CoinPackedMatrix matrix(false, 0, 0); 
    matrix.setDimensions(0, numVariables);
    
    vector<double> rowLowerVec;
    vector<double> rowUpperVec;

    // A. Restricciones de Salida y Entrada (Clientes)
    for (int i = 1; i < numClients; ++i) {
        vector<int> indicesOut, indicesIn; 
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

    // B. Bodega (K vehículos)
    vector<int> depotOutIdx, depotInIdx; 
    vector<double> depotOutEl, depotInEl;
    for (int j = 1; j < numClients; ++j) {
        depotOutIdx.push_back(getVarIndex(0, j)); depotOutEl.push_back(1.0);
        depotInIdx.push_back(getVarIndex(j, 0));  depotInEl.push_back(1.0);
    }
    matrix.appendRow(depotOutIdx.size(), depotOutIdx.data(), depotOutEl.data());
    rowLowerVec.push_back(numVehicles); rowUpperVec.push_back(numVehicles);

    matrix.appendRow(depotInIdx.size(), depotInIdx.data(), depotInEl.data());
    rowLowerVec.push_back(numVehicles); rowUpperVec.push_back(numVehicles);

    // C. MEJORA 2: Lifted MTZ (Desrochers & Laporte)
    for (int i = 1; i < numClients; ++i) {
        for (int j = 1; j < numClients; ++j) {
            if (i != j) {
                int di = parserData->getClients()[i + 1].getDemand();
                int dj = parserData->getClients()[j + 1].getDemand();
                
                // Incluimos X_ij y X_ji en la misma inecuación
                vector<int> indices = {getUIndex(i), getUIndex(j), getVarIndex(i, j), getVarIndex(j, i)};
                vector<double> elements = {1.0, -1.0, (double)Q, (double)(Q - di - dj)};
                double rhs = Q - dj;
                
                matrix.appendRow(indices.size(), indices.data(), elements.data());
                rowLowerVec.push_back(-COIN_DBL_MAX); rowUpperVec.push_back(rhs);
            }
        }
    }

    // D. MEJORA 3: Prohibición de 2-ciclos
    for (int i = 1; i < numClients; ++i) {
        for (int j = i + 1; j < numClients; ++j) {
            vector<int> indices = {getVarIndex(i, j), getVarIndex(j, i)};
            vector<double> elements = {1.0, 1.0};
            matrix.appendRow(indices.size(), indices.data(), elements.data());
            rowLowerVec.push_back(-COIN_DBL_MAX); rowUpperVec.push_back(1.0);
        }
    }

    // Cargamos todo el problema en OSI
    osi.loadProblem(matrix, colLower, colUpper, objective, rowLowerVec.data(), rowUpperVec.data());

    // 4. INTEGRALIDAD
    for (int i = 0; i < numClients; ++i) {
        for (int j = 0; j < numClients; ++j) {
            osi.setInteger(getVarIndex(i, j));
        }
    }

    // 5. Instanciamos CBC Model
    CbcModel model(osi);
    model.setLogLevel(1); 
    
    // --- NUEVO: CONFIGURACIÓN EXTREMA DE CBC ---
    
    // A. Warm Start: Inyectamos tu heurística constructiva (Clarke-Wright)
    GreedyBuilder builder(parserData);
    Solution initialSol = builder.buildSolution(); // 
    KOpt optimizador(parserData);
    Solution refinedInitialSol = optimizador.optimize(initialSol); // 
    
    double* mipStart = new double[numVariables];
    std::fill(mipStart, mipStart + numVariables, 0.0);
    
    // Mapeamos las rutas OPTIMIZADAS a variables binarias 1.0
    for (const auto& route : refinedInitialSol.getRoutes()) {
        const auto& path = route.getPath();
        for (size_t i = 0; i < path.size() - 1; ++i) {
            mipStart[getVarIndex(path[i] - 1, path[i+1] - 1)] = 1.0;
        }
    }
    
    // Le entregamos el 829 a CBC
    model.setBestSolution(mipStart, numVariables, refinedInitialSol.getTotalCost());
    model.setCutoff(refinedInitialSol.getTotalCost());

    // B. Prioridad de Ramificación: Forzamos a CBC a ramificar X_ij antes que U_i
    int* priorities = new int[numVariables];
    for(int i = 0; i < numVariables; i++) priorities[i] = 1000; // Prioridad baja por defecto
    for (int i = 0; i < numClients; ++i) {
        for (int j = 0; j < numClients; ++j) {
            priorities[getVarIndex(i, j)] = 1; // Prioridad alta para las rutas
        }
    }
    model.passInPriorities(priorities, false);

    // C. Ajustes de agresividad para Heurísticas y Cortes internos
    model.setMaximumNodes(100000); 
    
    // -------------------------------------------

    cout << "\n=== INICIANDO CBC BRANCH & CUT AUTOMATICO ===" << endl;
    cout << "Costo Heuristico Inicial Inyectado: " << initialSol.getTotalCost() << endl;
    
    model.branchAndBound(); 

    Solution bestSolution(parserData);

    if (model.isProvenOptimal() || model.bestSolution() != nullptr) {
        cout << "\nCBC finalizo. Costo MIP Encontrado: " << model.getObjValue() << endl;
        bestSolution = convertToSolution(model.bestSolution());
        
        // Pulimos la salida final por si CBC encontró algo bueno pero ligeramente cruzado
        bestSolution = optimizador.optimize(bestSolution);
        cout << "Costo Final tras refinamiento local (3-OPT): " << bestSolution.getTotalCost() << endl;
    } else {
        cout << "\nCBC no pudo encontrar solucion." << endl;
    }

    delete[] objective;
    delete[] colLower;
    delete[] colUpper;
    delete[] mipStart;
    delete[] priorities;

    return bestSolution;
}