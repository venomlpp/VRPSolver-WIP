#include "BranchAndBound.h"
#include <iostream>
#include <cmath>
#include <coin/CoinBuild.hpp>

using namespace std;

BranchAndBound::BranchAndBound(const Parser* parser, const Solution& initialSolution) 
    : parserData(parser), bestSolution(initialSolution) {
    globalUpperBound = initialSolution.getTotalCost();
    numClients = parserData->getDimension(); 
    numVariables = numClients * numClients; // Adiós a las variables Ui
}

int BranchAndBound::getVarIndex(int i, int j) const {
    return i * numClients + j;
}

// MODELO BASE: Ultra rápido, solo grados de entrada y salida
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
    build.addRow(depotOutIdx.size(), depotOutIdx.data(), depotOutEl.data(), K, K);
    build.addRow(depotInIdx.size(), depotInIdx.data(), depotInEl.data(), K, K);

    // Prohibir 2-ciclos: X_ij + X_ji <= 1
    for (int i = 1; i < numClients; ++i) {
        for (int j = i + 1; j < numClients; ++j) {
            vector<int> indices = {getVarIndex(i, j), getVarIndex(j, i)};
            vector<double> elements = {1.0, 1.0};
            // La suma no puede exceder 1.0
            build.addRow(indices.size(), indices.data(), elements.data(), -COIN_DBL_MAX, 1.0);
        }
    }

    model.addRows(build);
}

// EL DETECTOR DE TRAMPAS: Encuentra subtours aislados y rutas que violan capacidad
vector<vector<int>> BranchAndBound::findInvalidSets(const double* solution) const {
    vector<vector<int>> invalidSets;
    vector<bool> visited(numClients, false);
    visited[0] = true;

    // 1. Rastrear rutas conectadas a la bodega (Verificamos CAPACIDAD)
    for (int j = 1; j < numClients; ++j) {
        if (solution[getVarIndex(0, j)] > 0.5) {
            vector<int> currentRoute;
            int currentLoad = 0;
            int curr = j;
            
            while (curr != 0) {
                visited[curr] = true;
                currentRoute.push_back(curr);
                // Ajuste +1 porque el parser usa IDs desde 1
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
            
            // Si la ruta sobrepasa la carga Q, necesita ser cortada
            if (currentLoad > parserData->getCapacity()) {
                invalidSets.push_back(currentRoute);
            }
        }
    }

    // 2. Rastrear islas fantasmas (Verificamos SUBTOURS aislados)
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

int BranchAndBound::getMostFractionalVariable(const double* solution) const {
    int bestVar = -1;
    double maxImpact = 0.0;
    
    // Necesitamos acceder a la función objetivo (las distancias)
    // Asumiendo que tu modelo base o parser puede darnos el costo de la variable 'i'
    for (int i = 0; i < numVariables; ++i) {
        double val = solution[i];
        double fractionality = std::abs(val - std::round(val));
        
        if (fractionality > 0.01) {
            // Extraemos i, j para saber la distancia
            int from = i / numClients;
            int to = i % numClients;
            double distance = parserData->getDistance(from + 1, to + 1);
            
            // EL TRUCO: Impacto = Fraccionalidad * Costo del viaje
            double impact = fractionality * distance; 
            
            if (impact > maxImpact) {
                maxImpact = impact;
                bestVar = i;
            }
        }
    }
    return bestVar; 
}

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

Solution BranchAndBound::roundingHeuristic(const double* solution) const {
    Solution heuristicSol(parserData);
    int capacity = parserData->getCapacity();
    std::vector<bool> visited(numClients, false);
    visited[0] = true; // La bodega siempre está visitada

    // Buscamos iniciar camiones desde la bodega
    for (int j = 1; j < numClients; ++j) {
        // Tu regla: Si CLP sugiere fuertemente salir de la bodega hacia 'j' (> 0.5 o > 0.8)
        if (solution[getVarIndex(0, j)] > 0.5 && !visited[j]) {
            Route route(capacity, parserData);
            int curr = j;
            
            while (curr != 0) {
                route.addClient(curr + 1);
                visited[curr] = true;
                
                int next = -1;
                double bestVal = -1.0;
                
                // Buscamos el siguiente salto priorizando los decimales más altos de CLP
                for (int k = 0; k < numClients; ++k) {
                    if (!visited[k] || k == 0) { // Podemos ir a un no visitado o volver a la bodega
                        double val = solution[getVarIndex(curr, k)];
                        // Tomamos el camino que CLP recomienda con más fuerza
                        if (val > bestVal) {
                            bestVal = val;
                            next = k;
                        }
                    }
                }
                
                // Si no hay salida lógica (o CLP no sugiere nada fuerte), forzamos regreso a bodega
                if (next == -1 || (bestVal < 0.2 && next != 0)) {
                    curr = 0; 
                } else {
                    curr = next;
                }
            }
            heuristicSol.addRoute(route);
        }
    }
    
    // Verificación de seguridad: ¿Quedaron clientes sin visitar?
    // Si la heurística dejó clientes abandonados, forzamos camiones directos (Bodega -> Cliente -> Bodega)
    // para que la solución sea válida, aunque sea cara. El 3-OPT se encargará de arreglarlo después.
    for (int i = 1; i < numClients; ++i) {
        if (!visited[i]) {
            Route emergencyRoute(capacity, parserData);
            emergencyRoute.addClient(i + 1);
            heuristicSol.addRoute(emergencyRoute);
        }
    }
    
    return heuristicSol;
}

Solution BranchAndBound::solveBestFirst() {
    ClpSimplex baseModel;
    baseModel.setLogLevel(0); 
    buildBaseModel(baseModel);

    priority_queue<BBNode> pq;

    BBNode root;
    root.level = 0;
    root.lowerBounds.assign(numVariables, 0.0);
    root.upperBounds.assign(numVariables, 1.0); 
    
    baseModel.initialSolve();
    cout << "LB para el modelo inicial: " << baseModel.objectiveValue() << " | UB inicial (Heurística): " << globalUpperBound << endl;
    if (!baseModel.isProvenOptimal()) return bestSolution; 
    
    root.lowerBound = baseModel.objectiveValue();
    pq.push(root);

    int nodesExplored = 0;
    int cutsAdded = 0;

    while (!pq.empty()) {
        int MAX_NODES = 50000;
        BBNode current = pq.top();
        pq.pop();
        nodesExplored++;
        if (nodesExplored >= MAX_NODES) {
            cout << "Límite de nodos explorados alcanzado (" << MAX_NODES << "). Deteniendo DFS." << endl;
            break;
        }

        if (nodesExplored % 1000 == 0) {
            cout << "[Debug B&B] Nodos: " << nodesExplored 
                 << " | Cola: " << pq.size() 
                 << " | Cortes Inyectados: " << cutsAdded
                 << " | Mejor LB: " << current.lowerBound 
                 << " | UB: " << globalUpperBound << endl;
        }
        
        // Poda Bounding
        if (current.lowerBound >= globalUpperBound) continue; 

        // Aplicar límites del nodo
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
        //Heurística por redondeo
        Solution roundedSol = roundingHeuristic(solution);
        if (roundedSol.isValid()) {
            // Le pasamos tu 3-OPT para que refine los errores del redondeo
            KOpt optimizador(parserData);
            Solution refinedSol = optimizador.optimize(roundedSol);
            
            if (refinedSol.getTotalCost() < globalUpperBound) {
                globalUpperBound = refinedSol.getTotalCost();
                bestSolution = refinedSol;
                cout << "\n[HEURÍSTICA DE REDONDEO] ¡BINGO! Nuevo UB encontrado: " 
                     << globalUpperBound << " en el nodo " << nodesExplored << "\n" << endl;
            }
        }

        // --- EL CORAZÓN DE DFJ (LAZY CONSTRAINTS) ---
        if (fractionalVar == -1) { 
            // CLP entregó una solución de 0s y 1s. ¡A buscar trampas!
            vector<vector<int>> invalidSets = findInvalidSets(solution);
            
            if (invalidSets.empty()) {
                // No hay trampas. Es una solución legal CVRP.
                if (currentObj < globalUpperBound) {
                    globalUpperBound = currentObj;
                    bestSolution = convertToSolution(solution);
                    cout << "[DFJ] Nueva Cota Superior (UB): " << globalUpperBound 
                         << " en nodo " << nodesExplored << " | Cortes acumulados: " << cutsAdded << endl;
                }
            } else {
                // Trampa detectada. Inyectamos la inecuación a la matriz global.
                CoinBuild cutBuilder;
                for (const auto& S : invalidSets) {
                    int demandSum = 0;
                    for (int node : S) demandSum += parserData->getClients()[node + 1].getDemand();
                    
                    // Vehículos mínimos necesarios para este set (Capacity Cut)
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
                    // La suma de viajes internos debe ser <= |S| - K_s
                    cutBuilder.addRow(indices.size(), indices.data(), elements.data(), -COIN_DBL_MAX, S.size() - k_s);
                    cutsAdded++;
                }
                baseModel.addRows(cutBuilder);
                
                // La matriz cambió. Re-encolamos ESTE MISMO NODO para que CLP lo resuelva
                // obligatoriamente respetando la nueva ley matemática.
                current.lowerBound = currentObj; 
                pq.push(current);
            }
            continue; 
        }

        // Ramificación normal si es fraccionario
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

    cout << "Arbol explorado. Nodos: " << nodesExplored << " | Cortes inyectados: " << cutsAdded << endl;
    return bestSolution;
}

Solution BranchAndBound::solveDepthFirst() {
    ClpSimplex baseModel;
    baseModel.setLogLevel(0); 
    buildBaseModel(baseModel);

    // CAMBIO CLAVE: Usamos una Pila (Stack) para DFS en lugar de Cola de Prioridad
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
    int MAX_NODES = 20000;
    int cutsAdded = 0;

    while (!st.empty()) {
        // En una pila también usamos top() para leer y pop() para extraer
        if (nodesExplored >= MAX_NODES) {
            cout << "Límite de nodos explorados alcanzado (" << MAX_NODES << "). Deteniendo DFS." << endl;
            break;
        }
        BBNode current = st.top();
        st.pop();
        nodesExplored++;
        

        if (nodesExplored % 1000 == 0) {
            cout << "[Debug B&B (DFS)] Nodos: " << nodesExplored 
                 << " | Pila: " << st.size() 
                 << " | Cortes Inyectados: " << cutsAdded
                 << " | LB de este nodo: " << current.lowerBound 
                 << " | UB Actual: " << globalUpperBound << endl;
        }
        
        // Poda Bounding
        if (current.lowerBound >= globalUpperBound) continue; 

        // Aplicar límites del nodo
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
        //Heurística por redondeo
        Solution roundedSol = roundingHeuristic(solution);
        if (roundedSol.isValid()) {
            // Le pasamos tu 3-OPT para que refine los errores del redondeo
            KOpt optimizador(parserData);
            Solution refinedSol = optimizador.optimize(roundedSol);
            
            if (refinedSol.getTotalCost() < globalUpperBound) {
                globalUpperBound = refinedSol.getTotalCost();
                bestSolution = refinedSol;
                cout << "\n[HEURÍSTICA DE REDONDEO] ¡BINGO! Nuevo UB encontrado: " 
                     << globalUpperBound << " en el nodo " << nodesExplored << "\n" << endl;
            }
        }


        // --- EL CORAZÓN DE DFJ (LAZY CONSTRAINTS) ---
        if (fractionalVar == -1) { 
            // CLP entregó una solución de 0s y 1s. ¡A buscar trampas!
            vector<vector<int>> invalidSets = findInvalidSets(solution);
            
            if (invalidSets.empty()) {
                // No hay trampas. Es una solución legal CVRP.
                if (currentObj < globalUpperBound) {
                    globalUpperBound = currentObj;
                    bestSolution = convertToSolution(solution);
                    cout << "[DFJ - DFS] Nueva Cota Superior (UB): " << globalUpperBound 
                         << " en nodo " << nodesExplored << " | Cortes acumulados: " << cutsAdded << endl;
                }
            } else {
                // Trampa detectada. Inyectamos la inecuación a la matriz global.
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

        // Ramificación normal si es fraccionario
        BBNode leftChild = current;
        leftChild.level = current.level + 1;
        leftChild.upperBounds[fractionalVar] = 0.0;
        leftChild.lowerBound = currentObj; 

        BBNode rightChild = current;
        rightChild.level = current.level + 1;
        rightChild.lowerBounds[fractionalVar] = 1.0;
        rightChild.lowerBound = currentObj; 

        // NUEVO: Value Branching (Guiando al DFS)
        double fractionalVal = solution[fractionalVar];
        
        // La Pila (LIFO) saca el ÚLTIMO elemento insertado primero.
        if (fractionalVal >= 0.5) {
            // CLP cree que esta arista DEBERÍA existir.
            st.push(leftChild);  // Metemos x=0 al fondo (para después)
            st.push(rightChild); // Metemos x=1 arriba (se evalúa INMEDIATAMENTE)
        } else {
            // CLP cree que esta arista NO debería existir.
            st.push(rightChild); // Metemos x=1 al fondo (para después)
            st.push(leftChild);  // Metemos x=0 arriba (se evalúa INMEDIATAMENTE)
        }
    }

    cout << "Arbol explorado (DFS). Nodos: " << nodesExplored << " | Cortes inyectados: " << cutsAdded << endl;
    return bestSolution;
}

BranchAndBound::~BranchAndBound() {}