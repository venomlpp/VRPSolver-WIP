#include "ClpSolver.h"
#include <iostream>
#include <iomanip> // Para std::setprecision
#include <coin/CoinBuild.hpp>

using namespace std;

ClpSolver::ClpSolver(const Parser* parser) : parserData(parser) {
    numClients = parserData->getDimension();
    numVariables = numClients * numClients;
}

int ClpSolver::getVarIndex(int i, int j) const {
    return i * numClients + j;
}

void ClpSolver::solveRelaxation(int numVehicles) {
    ClpSimplex model;
    model.setLogLevel(0); // 0 = Silencioso, 1 = Prints de COIN-OR
    
    // 1. Configurar Columnas (Variables binarias relajadas entre 0 y 1)
    model.resize(0, numVariables);
    double* objective = model.objective();
    double* colLower = model.columnLower();
    double* colUpper = model.columnUpper();

    for (int i = 0; i < numClients; ++i) {
        for (int j = 0; j < numClients; ++j) {
            int idx = getVarIndex(i, j);
            objective[idx] = parserData->getDistance(i + 1, j + 1); 
            colLower[idx] = 0.0; 
            colUpper[idx] = (i == j) ? 0.0 : 1.0; 
        }
    }

    // 2. Construir Filas (Solo grados In/Out)
    CoinBuild build;
    
    for (int i = 1; i < numClients; ++i) {
        vector<int> indices; vector<double> elements;
        for (int j = 0; j < numClients; ++j) {
            if (i != j) { indices.push_back(getVarIndex(i, j)); elements.push_back(1.0); }
        }
        build.addRow(indices.size(), indices.data(), elements.data(), 1.0, 1.0);
    }

    for (int j = 1; j < numClients; ++j) {
        vector<int> indices; vector<double> elements;
        for (int i = 0; i < numClients; ++i) {
            if (i != j) { indices.push_back(getVarIndex(i, j)); elements.push_back(1.0); }
        }
        build.addRow(indices.size(), indices.data(), elements.data(), 1.0, 1.0);
    }

    vector<int> depotOutIdx; vector<double> depotOutEl;
    vector<int> depotInIdx; vector<double> depotInEl;
    for (int j = 1; j < numClients; ++j) {
        depotOutIdx.push_back(getVarIndex(0, j)); depotOutEl.push_back(1.0);
        depotInIdx.push_back(getVarIndex(j, 0)); depotInEl.push_back(1.0);
    }
    build.addRow(depotOutIdx.size(), depotOutIdx.data(), depotOutEl.data(), numVehicles, numVehicles);
    build.addRow(depotInIdx.size(), depotInIdx.data(), depotInEl.data(), numVehicles, numVehicles);

    model.addRows(build);

    // 3. Ejecutar el solver
    cout << "Resolviendo raiz continua (Sin B&B)..." << endl;
    model.initialSolve();

    if (model.isProvenOptimal()) {
        cout << "Optimo Continuo Alcanzado. Costo (Lower Bound absoluto): " 
             << model.objectiveValue() << endl;
        
        cout << "\n--- Variables Activas (x_ij > 0.01) ---" << endl;
        const double* solution = model.primalColumnSolution();
        for (int i = 0; i < numClients; ++i) {
            for (int j = 0; j < numClients; ++j) {
                int idx = getVarIndex(i, j);
                if (solution[idx] > 0.01) {
                    cout << "Viaje " << i+1 << " -> " << j+1 
                         << " : " << fixed << setprecision(3) << solution[idx] << endl;
                }
            }
        }
    } else {
        cout << "El modelo resulto infactible." << endl;
    }
}