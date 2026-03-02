#include "BranchAndBound.h"
#include <iostream>
#include <cmath>
#include <coin/CoinBuild.hpp>
#include <chrono>

using namespace std;

/*
 * Descripción: Constructor del algoritmo exacto.
 * Entrada: Puntero al parser, solución inicial factible (Warm Start).
 * Salida: Instancia inicializada.
 */
BranchAndBound::BranchAndBound(const Parser* parser, const Solution& initialSolution) 
    : parserData(parser), bestSolution(initialSolution) {
    globalUpperBound = initialSolution.getTotalCost();
    numClients = parserData->getDimension(); 
    numVariables = numClients * numClients; 
}

/*
 * Descripción: Mapea coordenadas 2D (origen, destino) a un índice lineal 1D.
 * Entrada: Índices de nodo i, j.
 * Salida: Índice de la variable X_ij en el modelo de CLP.
 */
int BranchAndBound::getVarIndex(int i, int j) const {
    return i * numClients + j;
}

/*
 * Descripción: Construye la relajación LP inicial. Solo incluye restricciones 
 * de grado (entrada/salida) y prohibición de 2-ciclos.
 * Entrada: Referencia al modelo ClpSimplex.
 * Salida: Ninguna (modifica el modelo in-place).
 */
void BranchAndBound::buildBaseModel(ClpSimplex& model) {
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

    CoinBuild build;
    int K = bestSolution.getRoutes().size(); 

    // Grado de salida = 1 para cada cliente
    for (int i = 1; i < numClients; ++i) {
        vector<int> indices; vector<double> elements;
        for (int j = 0; j < numClients; ++j) {
            if (i != j) { indices.push_back(getVarIndex(i, j)); elements.push_back(1.0); }
        }
        build.addRow(indices.size(), indices.data(), elements.data(), 1.0, 1.0);
    }

    // Grado de entrada = 1 para cada cliente
    for (int j = 1; j < numClients; ++j) {
        vector<int> indices; vector<double> elements;
        for (int i = 0; i < numClients; ++i) {
            if (i != j) { indices.push_back(getVarIndex(i, j)); elements.push_back(1.0); }
        }
        build.addRow(indices.size(), indices.data(), elements.data(), 1.0, 1.0);
    }

    // Grado de entrada y salida de la bodega = K vehículos
    vector<int> depotOutIdx; vector<double> depotOutEl;
    vector<int> depotInIdx; vector<double> depotInEl;
    for (int j = 1; j < numClients; ++j) {
        depotOutIdx.push_back(getVarIndex(0, j)); depotOutEl.push_back(1.0);
        depotInIdx.push_back(getVarIndex(j, 0)); depotInEl.push_back(1.0);
    }
    build.addRow(depotOutIdx.size(), depotOutIdx.data(), depotOutEl.data(), K, K);
    build.addRow(depotInIdx.size(), depotInIdx.data(), depotInEl.data(), K, K);

    // Restricción anti 2-ciclos (X_ij + X_ji <= 1)
    for (int i = 1; i < numClients; ++i) {
        for (int j = i + 1; j < numClients; ++j) {
            vector<int> indices = {getVarIndex(i, j), getVarIndex(j, i)};
            vector<double> elements = {1.0, 1.0};
            build.addRow(indices.size(), indices.data(), elements.data(), -COIN_DBL_MAX, 1.0);
        }
    }

    model.addRows(build);
}

/*
 * Descripción: Analiza una solución candidata para detectar conjuntos de nodos que forman
 * subtours aislados o exceden la capacidad vehicular máxima.
 * Entrada: Arreglo de variables de la solución LP.
 * Salida: Lista de conjuntos inválidos (S) para aplicar cortes DFJ.
 */
vector<vector<int>> BranchAndBound::findInvalidSets(const double* solution) const {
    vector<vector<int>> invalidSets;
    vector<bool> visited(numClients, false);
    visited[0] = true;

    // Verificar violaciones de capacidad en rutas conectadas a la bodega
    for (int j = 1; j < numClients; ++j) {
        if (solution[getVarIndex(0, j)] > 0.5) {
            vector<int> currentRoute;
            int currentLoad = 0;
            int curr = j;
            
            while (curr != 0) {
                visited[curr] = true;
                currentRoute.push_back(curr);
                currentLoad += parserData->getClients()[curr + 1].getDemand(); 
                
                int next = -1;
                for (int k = 0; k < numClients; ++k) {
                    if (curr != k && solution[getVarIndex(curr, k)] > 0.5) {
                        next = k; break;
                    }
                }
                if (next == -1) break; 
                curr = next;
            }
            
            if (currentLoad > parserData->getCapacity()) {
                invalidSets.push_back(currentRoute);
            }
        }
    }

    // Verificar subtours aislados (clientes no alcanzados desde la bodega)
    for (int i = 1; i < numClients; ++i) {
        if (!visited[i]) {
            vector<int> isolatedSet;
            int curr = i;
            while (!visited[curr]) {
                visited[curr] = true;
                isolatedSet.push_back(curr);
                
                int next = -1;
                for (int k = 1; k < numClients; ++k) {
                    if (curr != k && solution[getVarIndex(curr, k)] > 0.5) {
                        next = k; break;
                    }
                }
                if (next != -1) curr = next;
            }
            if (!isolatedSet.empty()) invalidSets.push_back(isolatedSet);
        }
    }
    return invalidSets;
}

/*
 * Descripción: Determina la variable fraccionaria de mayor impacto en la función objetivo.
 * Entrada: Arreglo de variables de la solución LP.
 * Salida: Índice de la variable seleccionada para ramificar.
 */
int BranchAndBound::getMostFractionalVariable(const double* solution) const {
    int bestVar = -1;
    double maxImpact = 0.0;
    
    for (int i = 0; i < numVariables; ++i) {
        double val = solution[i];
        double fractionality = std::abs(val - std::round(val));
        
        if (fractionality > 0.01) {
            int from = i / numClients;
            int to = i % numClients;
            double distance = parserData->getDistance(from + 1, to + 1);
            
            // Ponderamos fraccionalidad por el costo del arco
            double impact = fractionality * distance; 
            
            if (impact > maxImpact) {
                maxImpact = impact;
                bestVar = i;
            }
        }
    }
    return bestVar; 
}

/*
 * Descripción: Convierte un vector LP estrictamente binario en un objeto Solution.
 * Entrada: Arreglo de variables de la solución entera.
 * Salida: Objeto Solution formateado.
 */
Solution BranchAndBound::convertToSolution(const double* solution) const {
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
                        next = k; break;
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
 * Descripción: Construye una solución factible redondeando heurísticamente valores LP.
 * Entrada: Arreglo de variables continuas LP.
 * Salida: Solución factible generada.
 */
Solution BranchAndBound::roundingHeuristic(const double* solution) const {
    Solution heuristicSol(parserData);
    int capacity = parserData->getCapacity();
    std::vector<bool> visited(numClients, false);
    visited[0] = true; 

    for (int j = 1; j < numClients; ++j) {
        if (solution[getVarIndex(0, j)] > 0.5 && !visited[j]) {
            Route route(capacity, parserData);
            int curr = j;
            
            while (curr != 0) {
                route.addClient(curr + 1);
                visited[curr] = true;
                
                int next = -1;
                double bestVal = -1.0;
                
                for (int k = 0; k < numClients; ++k) {
                    if (!visited[k] || k == 0) { 
                        double val = solution[getVarIndex(curr, k)];
                        if (val > bestVal) {
                            bestVal = val;
                            next = k;
                        }
                    }
                }
                
                if (next == -1 || (bestVal < 0.2 && next != 0)) {
                    curr = 0; 
                } else {
                    curr = next;
                }
            }
            heuristicSol.addRoute(route);
        }
    }
    
    // Forzar la recolección de nodos abandonados
    for (int i = 1; i < numClients; ++i) {
        if (!visited[i]) {
            Route emergencyRoute(capacity, parserData);
            emergencyRoute.addClient(i + 1);
            heuristicSol.addRoute(emergencyRoute);
        }
    }
    
    return heuristicSol;
}

/*
 * Descripción: Construye una solución basada en las preferencias fraccionarias del LP.
 * Entrada: Arreglo de variables continuas LP.
 * Salida: Solución factible armada.
 */
Solution BranchAndBound::lpGuidedConstruction(const double* lpSol) const {
    int Q   = parserData->getCapacity();
    int K   = (int)bestSolution.getRoutes().size();

    vector<int> clientOrder;
    for (int j = 1; j < numClients; j++)
        clientOrder.push_back(j);
        
    sort(clientOrder.begin(), clientOrder.end(), [&](int a, int b) {
        return lpSol[getVarIndex(0, a)] > lpSol[getVarIndex(0, b)];
    });

    vector<bool> assigned(numClients, false);
    assigned[0] = true;

    vector<vector<int>> routeClients(K);
    vector<int> routeLoad(K, 0);
    int started = 0;
    
    for (int j : clientOrder) {
        if (started >= K) break;
        routeClients[started].push_back(j);
        routeLoad[started] += parserData->getClients()[j+1].getDemand();
        assigned[j] = true;
        started++;
    }

    for (int j : clientOrder) {
        if (assigned[j]) continue;
        int demand = parserData->getClients()[j+1].getDemand();

        int bestRoute = -1;
        double bestScore = -1.0;

        for (int r = 0; r < K; r++) {
            if (routeLoad[r] + demand > Q) continue;
            for (int node : routeClients[r]) {
                double score = lpSol[getVarIndex(node, j)] + lpSol[getVarIndex(j, node)]; 
                if (score > bestScore) {
                    bestScore = score;
                    bestRoute = r;
                }
            }
        }

        if (bestRoute != -1) {
            routeClients[bestRoute].push_back(j);
            routeLoad[bestRoute] += demand;
        } else {
            routeClients.push_back({j});
            routeLoad.push_back(demand);
        }
        assigned[j] = true;
    }

    Solution sol(parserData);
    for (const auto& clients : routeClients) {
        if (clients.empty()) continue;
        Route route(Q, parserData);
        for (int c : clients) route.addClient(c + 1);
        sol.addRoute(route);
    }
    return sol;
}

/*
 * Descripción: Estrategia de recorrido Best-First Search para el árbol B&B.
 * Expande el nodo con la cota inferior más prometedora.
 * Entrada: Tiempo límite de ejecución.
 * Salida: Mejor solución entera encontrada.
 */
Solution BranchAndBound::solveBestFirst(double timeLimitSeconds) {
    ClpSimplex baseModel;
    baseModel.setLogLevel(0); 
    buildBaseModel(baseModel);

    priority_queue<BBNode> pq;

    BBNode root;
    root.level = 0;
    root.lowerBounds.assign(numVariables, 0.0);
    root.upperBounds.assign(numVariables, 1.0); 
    
    baseModel.initialSolve();
    cout << "LB inicial: " << baseModel.objectiveValue()
         << " | UB inicial (heuristica): " << globalUpperBound << endl;
         
    if (!baseModel.isProvenOptimal()) return bestSolution;
    
    root.lowerBound = baseModel.objectiveValue();
    pq.push(root);

    int nodesExplored = 0;
    int cutsAdded = 0;
    auto startTime = chrono::steady_clock::now();

    while (!pq.empty()) {
        auto elapsed = chrono::duration<double>(
            chrono::steady_clock::now() - startTime).count();
        if (elapsed >= timeLimitSeconds) {
            cout << "Limite de tiempo alcanzado (" << timeLimitSeconds
                 << "s). Deteniendo BestFirst." << endl;
            break;
        }
        
        BBNode current = pq.top();
        pq.pop();
        nodesExplored++;

        if (nodesExplored % 2000 == 0) {
            cout << "Nodos: " << nodesExplored 
                 << " | Cola: " << pq.size() 
                 << " | Cortes Inyectados: " << cutsAdded
                 << " | Mejor LB: " << current.lowerBound 
                 << " | UB: " << globalUpperBound << endl;
        }
        
        if (current.lowerBound >= globalUpperBound) continue; 

        for(int i = 0; i < numVariables; ++i) {
            baseModel.setColumnLower(i, current.lowerBounds[i]);
            baseModel.setColumnUpper(i, current.upperBounds[i]);
        }

        baseModel.dual(); 

        if (!baseModel.isProvenOptimal()) continue; 
        double currentObj = baseModel.objectiveValue();
        if (currentObj >= globalUpperBound) continue;

        const double* solution = baseModel.primalColumnSolution();
        int fractionalVar = getMostFractionalVariable(solution);
        
        Solution roundedSol = lpGuidedConstruction(solution);
        if (roundedSol.isValid()) {
            KOpt kopt(parserData);
            Solution koptSol = kopt.optimize(roundedSol);
    
            if (koptSol.getTotalCost() < globalUpperBound) {
                globalUpperBound = koptSol.getTotalCost();
                bestSolution = koptSol;
                cout << "[HEURISTICA] Nuevo UB: " << globalUpperBound
                    << " en nodo " << nodesExplored << endl;
            }
        }

        if (fractionalVar == -1) { 
            vector<vector<int>> invalidSets = findInvalidSets(solution);
            
            if (invalidSets.empty()) {
                if (currentObj < globalUpperBound) {
                    globalUpperBound = currentObj;
                    bestSolution = convertToSolution(solution);
                    cout << "[DFJ] Nueva Cota Superior (UB): " << globalUpperBound 
                         << " en nodo " << nodesExplored << " | Cortes acumulados: " << cutsAdded << endl;
                }
            } else {
                CoinBuild cutBuilder;
                for (const auto& S : invalidSets) {
                    int demandSum = 0;
                    for (int node : S) demandSum += parserData->getClients()[node + 1].getDemand();
                    
                    int k_s = std::ceil((double)demandSum / parserData->getCapacity());
                    if (k_s < 1) k_s = 1; 
                    
                    vector<int> indices; vector<double> elements;
                    for (int u : S) {
                        for (int v : S) {
                            if (u != v) {
                                indices.push_back(getVarIndex(u, v));
                                elements.push_back(1.0);
                            }
                        }
                    }
                    cutBuilder.addRow(indices.size(), indices.data(), elements.data(), -COIN_DBL_MAX, S.size() - k_s);
                    cutsAdded++;
                }
                baseModel.addRows(cutBuilder);
                
                current.lowerBound = currentObj; 
                pq.push(current);
            }
            continue; 
        }

        BBNode leftChild = current;
        leftChild.level = current.level + 1;
        leftChild.upperBounds[fractionalVar] = 0.0;
        leftChild.lowerBound = currentObj; 
        pq.push(leftChild);

        BBNode rightChild = current;
        rightChild.level = current.level + 1;
        rightChild.lowerBounds[fractionalVar] = 1.0;
        rightChild.lowerBound = currentObj; 
        pq.push(rightChild);
    }

    cout << "BestFirst finalizado. Nodos: " << nodesExplored
         << " | Cortes: " << cutsAdded << endl;

    return bestSolution;
}

/*
 * Descripción: Estrategia de recorrido Depth-First Search para el árbol B&B.
 * Expande los nodos buscando factibilidad rápida, útil para memoria reducida.
 * Entrada: Tiempo límite de ejecución.
 * Salida: Mejor solución entera encontrada.
 */
Solution BranchAndBound::solveDepthFirst(double timeLimitSeconds) {
    ClpSimplex baseModel;
    baseModel.setLogLevel(0); 
    buildBaseModel(baseModel);

    std::stack<BBNode> st;

    BBNode root;
    root.level = 0;
    root.lowerBounds.assign(numVariables, 0.0);
    root.upperBounds.assign(numVariables, 1.0); 
    
    baseModel.initialSolve();
    if (!baseModel.isProvenOptimal()) return bestSolution; 
    
    root.lowerBound = baseModel.objectiveValue();
    st.push(root);

    int nodesExplored = 0;
    int cutsAdded = 0;
    auto startTime = chrono::steady_clock::now();

    while (!st.empty()) {
        BBNode current = st.top();
        st.pop();
        nodesExplored++;
        
        auto elapsed = chrono::duration<double>(
            chrono::steady_clock::now() - startTime).count();
        if (elapsed >= timeLimitSeconds) {
            cout << "Limite de tiempo alcanzado (" << timeLimitSeconds
                 << "s). Deteniendo DepthFirst." << endl;
            break;
        }

        if (nodesExplored % 2000 == 0) {
            cout << "Nodos: " << nodesExplored 
                 << " | Pila: " << st.size() 
                 << " | Cortes Inyectados: " << cutsAdded
                 << " | LB de este nodo: " << current.lowerBound 
                 << " | UB Actual: " << globalUpperBound << endl;
        }
        
        if (current.lowerBound >= globalUpperBound) continue; 

        for(int i = 0; i < numVariables; ++i) {
            baseModel.setColumnLower(i, current.lowerBounds[i]);
            baseModel.setColumnUpper(i, current.upperBounds[i]);
        }

        baseModel.dual(); 

        if (!baseModel.isProvenOptimal()) continue; 
        double currentObj = baseModel.objectiveValue();
        if (currentObj >= globalUpperBound) continue;

        const double* solution = baseModel.primalColumnSolution();
        int fractionalVar = getMostFractionalVariable(solution);
        
        Solution roundedSol = lpGuidedConstruction(solution);
        if (roundedSol.isValid()) {
            KOpt kopt(parserData);
            Solution koptSol = kopt.optimize(roundedSol);
    
            if (koptSol.getTotalCost() < globalUpperBound) {
                globalUpperBound = koptSol.getTotalCost();
                bestSolution = koptSol;
                cout << "[HEURISTICA] Nuevo UB: " << globalUpperBound
                    << " en nodo " << nodesExplored << endl;
            }
        }

        if (fractionalVar == -1) { 
            vector<vector<int>> invalidSets = findInvalidSets(solution);
            
            if (invalidSets.empty()) {
                if (currentObj < globalUpperBound) {
                    globalUpperBound = currentObj;
                    bestSolution = convertToSolution(solution);
                    cout << "[DFJ - DFS] Nueva Cota Superior (UB): " << globalUpperBound 
                         << " en nodo " << nodesExplored << " | Cortes acumulados: " << cutsAdded << endl;
                }
            } else {
                CoinBuild cutBuilder;
                for (const auto& S : invalidSets) {
                    int demandSum = 0;
                    for (int node : S) demandSum += parserData->getClients()[node + 1].getDemand();
                    
                    int k_s = std::ceil((double)demandSum / parserData->getCapacity());
                    if (k_s < 1) k_s = 1; 
                    
                    vector<int> indices; vector<double> elements;
                    for (int u : S) {
                        for (int v : S) {
                            if (u != v) {
                                indices.push_back(getVarIndex(u, v));
                                elements.push_back(1.0);
                            }
                        }
                    }
                    cutBuilder.addRow(indices.size(), indices.data(), elements.data(), -COIN_DBL_MAX, S.size() - k_s);
                    cutsAdded++;
                }
                baseModel.addRows(cutBuilder);
                
                current.lowerBound = currentObj; 
                st.push(current);
            }
            continue; 
        }

        BBNode leftChild = current;
        leftChild.level = current.level + 1;
        leftChild.upperBounds[fractionalVar] = 0.0;
        leftChild.lowerBound = currentObj; 

        BBNode rightChild = current;
        rightChild.level = current.level + 1;
        rightChild.lowerBounds[fractionalVar] = 1.0;
        rightChild.lowerBound = currentObj; 

        // Value Branching: Insertar primero la rama contraria al valor continuo,
        // luego la rama esperada (para que la pila la evalúe inmediatamente).
        double fractionalVal = solution[fractionalVar];
        if (fractionalVal >= 0.5) {
            st.push(leftChild);  
            st.push(rightChild); 
        } else {
            st.push(rightChild); 
            st.push(leftChild);  
        }
    }

    cout << "DepthFirst finalizado. Nodos: " << nodesExplored
         << " | Cortes: " << cutsAdded << endl;
         
    return bestSolution;
}

BranchAndBound::~BranchAndBound() {}