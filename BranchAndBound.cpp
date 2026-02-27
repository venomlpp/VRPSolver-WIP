#include "BranchAndBound.h"
#include <iostream>
#include <cmath>
#include <coin/CoinBuild.hpp>

using namespace std;

// Constructor
BranchAndBound::BranchAndBound(const Parser* parser, const Solution& initialSolution) 
    : parserData(parser), bestSolution(initialSolution) {
    
    globalUpperBound = initialSolution.getTotalCost();
    numClients = parserData->getDimension(); 
    
    // SOLO Variables X_ij (N * N)
    numVariables = (numClients * numClients); 
}

// Mapeo 2D a 1D para X_ij (Asume índices de 0 a N-1)
int BranchAndBound::getVarIndex(int i, int j) const {
    return i * numClients + j;
}

// Mapeo para U_i (Solo para clientes 1 a N-1)
int getUIndex(int i, int numClients) {
    return (numClients * numClients) + (i - 1);
}

void BranchAndBound::buildBaseModel(ClpSimplex& model) {
    // 1. Configurar Columnas (Variables)
    model.resize(0, numVariables);
    double* objective = model.objective();
    double* colLower = model.columnLower();
    double* colUpper = model.columnUpper();

    // Configuración Variables X_ij
    for (int i = 0; i < numClients; ++i) {
        for (int j = 0; j < numClients; ++j) {
            int idx = getVarIndex(i, j);
            // El parser es 1-based, iteramos 0-based
            objective[idx] = parserData->getDistance(i + 1, j + 1); 
            
            if (i == j) {
                colLower[idx] = 0.0; colUpper[idx] = 0.0; // Sin auto-ciclos
            } else {
                colLower[idx] = 0.0; colUpper[idx] = 1.0; // Variable binaria (relajada continua)
            }
        }
    }

    // Configuración Variables U_i (Cargas)
    int Q = parserData->getCapacity();
    for (int i = 1; i < numClients; ++i) {
        int idx = getUIndex(i, numClients);
        objective[idx] = 0.0; // La carga no suma costo a la función objetivo
        
        int demand = parserData->getClients()[i + 1].getDemand();
        colLower[idx] = demand; 
        colUpper[idx] = Q;
    }

    // 2. Construir Filas (Restricciones)
    CoinBuild build;
    int K = bestSolution.getRoutes().size(); // Número de vehículos a usar

    // A. Restricciones de Salida (Out-degree)
    for (int i = 1; i < numClients; ++i) {
        vector<int> indices; vector<double> elements;
        for (int j = 0; j < numClients; ++j) {
            if (i != j) { indices.push_back(getVarIndex(i, j)); elements.push_back(1.0); }
        }
        build.addRow(indices.size(), indices.data(), elements.data(), 1.0, 1.0); // Suma = 1
    }

    // B. Restricciones de Entrada (In-degree)
    for (int j = 1; j < numClients; ++j) {
        vector<int> indices; vector<double> elements;
        for (int i = 0; i < numClients; ++i) {
            if (i != j) { indices.push_back(getVarIndex(i, j)); elements.push_back(1.0); }
        }
        build.addRow(indices.size(), indices.data(), elements.data(), 1.0, 1.0); // Suma = 1
    }

    // C. Salidas y Entradas de la Bodega (i = 0)
    vector<int> depotOutIdx; vector<double> depotOutEl;
    vector<int> depotInIdx; vector<double> depotInEl;
    for (int j = 1; j < numClients; ++j) {
        depotOutIdx.push_back(getVarIndex(0, j)); depotOutEl.push_back(1.0);
        depotInIdx.push_back(getVarIndex(j, 0)); depotInEl.push_back(1.0);
    }
    build.addRow(depotOutIdx.size(), depotOutIdx.data(), depotOutEl.data(), K, K);
    build.addRow(depotInIdx.size(), depotInIdx.data(), depotInEl.data(), K, K);

    // D. RESTRICCIONES MTZ (Anti-subtours)
    // u_i - u_j + Q * X_ij <= Q - q_j  (para i, j >= 1; i != j)
    for (int i = 1; i < numClients; ++i) {
        for (int j = 1; j < numClients; ++j) {
            if (i != j) {
                vector<int> indices; vector<double> elements;
                indices.push_back(getUIndex(i, numClients)); elements.push_back(1.0);      // u_i
                indices.push_back(getUIndex(j, numClients)); elements.push_back(-1.0);     // -u_j
                indices.push_back(getVarIndex(i, j));        elements.push_back(Q);        // +Q * x_ij
                
                int demandJ = parserData->getClients()[j + 1].getDemand();
                double rhs = Q - demandJ; // Lado derecho de la inecuación
                
                // Límites de la fila: [-Inf, rhs]
                build.addRow(indices.size(), indices.data(), elements.data(), -COIN_DBL_MAX, rhs); 
            }
        }
    }

     // Inyectamos todas las filas construidas al modelo
    model.addRows(build);
}

int BranchAndBound::getMostFractionalVariable(const double* solution) const {
    int bestVar = -1;
    double maxFractionality = 0.0;

    // Solo ramificamos sobre las variables X_ij (no sobre las U_i)
    for (int i = 0; i < numClients * numClients; ++i) {
        double val = solution[i];
        double fractionality = std::abs(val - std::round(val));
        
        // Buscamos el valor más cercano a 0.5 (max fractionality)
        if (fractionality > 0.01 && fractionality > maxFractionality) {
            maxFractionality = fractionality;
            bestVar = i;
        }
    }
    return bestVar; // Retorna -1 si todas son enteras
}

Solution BranchAndBound::convertToSolution(const double* solution) const {
    Solution newSolution(parserData);
    int capacity = parserData->getCapacity();

    // Buscamos todas las salidas desde la bodega (nodo 0 en CLP)
    for (int j = 1; j < numClients; ++j) {
        int outOfDepotIdx = getVarIndex(0, j);
        
        // Usamos > 0.5 por seguridad con la precisión de punto flotante de double
        if (solution[outOfDepotIdx] > 0.5) { 
            
            // Inicializamos un nuevo camión
            Route route(capacity, parserData);
            int currentNode = j;
            
            // Seguimos el rastro de 1s hasta volver a la bodega (nodo 0)
            while (currentNode != 0) {
                // Agregamos el cliente a la ruta (ajustando el índice al ID real)
                route.addClient(currentNode + 1);
                
                // Buscamos hacia dónde viaja el camión desde este nodo
                int nextNode = -1;
                for (int k = 0; k < numClients; ++k) {
                    if (currentNode != k) { // No buscamos auto-ciclos
                        int edgeIdx = getVarIndex(currentNode, k);
                        if (solution[edgeIdx] > 0.5) {
                            nextNode = k;
                            break; // Encontramos el siguiente salto, dejamos de buscar
                        }
                    }
                }
                
                // Nos movemos al siguiente nodo para continuar el ciclo while
                currentNode = nextNode;
            }
            
            // Guardamos la ruta terminada en la solución
            newSolution.addRoute(route);
        }
    }
    
    // Forzamos la actualización del costo total Z sumando las rutas
    // (Aunque Route.addClient ya lo hace internamente, es una buena práctica)
    return newSolution;
}

Solution BranchAndBound::solveBestFirst() {
    ClpSimplex baseModel;
    baseModel.setLogLevel(0); // Apagar prints de la consola de CLP
    buildBaseModel(baseModel);

    std::priority_queue<BBNode> pq;

    // Crear el nodo raíz
    BBNode root;
    root.level = 0;
    root.lowerBounds.assign(numVariables, 0.0);
    root.upperBounds.assign(numVariables, 1.0); // Nota: Las cotas de U_i se aplican directo en CLP
    
    // Resolver la raíz para obtener el primer Lower Bound
    baseModel.initialSolve();
    if (!baseModel.isProvenOptimal()) return bestSolution; 
    
    root.lowerBound = baseModel.objectiveValue();
    pq.push(root);

    int nodesExplored = 0;
    int MAX_NODES = 50000; // Límite de nodos para evitar exploración infinita (ajustable)

    while (!pq.empty()) {
        BBNode current = pq.top();
        pq.pop();
        nodesExplored++;

        // --- INICIO DEL MONITOR DE DEBUG ---
        // Imprime el estado cada 1000 nodos explorados
        if (nodesExplored % 1000 == 0) {
            cout << "[Debug B&B] Nodos: " << nodesExplored 
                 << " | En Cola: " << pq.size() 
                 << " | Mejor LB en progreso: " << current.lowerBound 
                 << " | UB Actual: " << globalUpperBound << endl;
        }

        // Cortocircuito de seguridad si el árbol explota
        if (nodesExplored >= MAX_NODES) {
            cout << "\n[ADVERTENCIA] Se alcanzo el limite maximo de " << MAX_NODES 
                 << " nodos explorados. Deteniendo B&B anticipadamente." << endl;
            break;
        }
        // --- FIN DEL MONITOR DE DEBUG ---

        // 1. Poda por Límite (Bounding de Taylor)
        if (current.lowerBound >= globalUpperBound) {
            continue; // Podamos: Este camino ya no puede superar nuestro récord
        }

        // Aplicamos las cotas fijadas de este nodo al modelo
        for(int i = 0; i < numClients * numClients; ++i) {
            baseModel.setColumnLower(i, current.lowerBounds[i]);
            baseModel.setColumnUpper(i, current.upperBounds[i]);
        }

        baseModel.dual(); // Resolvemos en caliente usando Dual Simplex (más rápido para B&B)

        // 2. Poda por Infactibilidad
        if (!baseModel.isProvenOptimal()) continue; 

        double currentObj = baseModel.objectiveValue();
        if (currentObj >= globalUpperBound) continue;

        const double* solution = baseModel.primalColumnSolution();
        int fractionalVar = getMostFractionalVariable(solution);

        // 3. Poda por Integralidad (Llegamos a una ruta real)
        if (fractionalVar == -1) { 
            // ¡Es una solución entera y válida!
            Solution newSolution = convertToSolution(solution);
            if (newSolution.isValid() && newSolution.getTotalCost() < globalUpperBound) {
                globalUpperBound = currentObj;
                bestSolution = newSolution;
            }
            cout << "[B&B] Nueva Cota Superior Encontrada: " << globalUpperBound << " en nodo " << nodesExplored << endl;
            
            continue; 
        }

        // 4. Ramificación (Branch)
        // Rama Izquierda: Forzamos la variable a 0
        BBNode leftChild = current;
        leftChild.level = current.level + 1;
        leftChild.upperBounds[fractionalVar] = 0.0;
        leftChild.lowerBound = currentObj; // Hereda el costo del padre provisionalmente
        pq.push(leftChild);

        // Rama Derecha: Forzamos la variable a 1
        BBNode rightChild = current;
        rightChild.level = current.level + 1;
        rightChild.lowerBounds[fractionalVar] = 1.0;
        rightChild.lowerBound = currentObj; 
        pq.push(rightChild);
    }

    cout << "Arbol explorado por completo. Nodos procesados: " << nodesExplored << endl;
    return bestSolution;
}

BranchAndBound::~BranchAndBound() {}