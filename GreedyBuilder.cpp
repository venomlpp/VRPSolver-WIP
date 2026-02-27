#include "GreedyBuilder.h"
#include <iostream>

using namespace std;

// Constructor
GreedyBuilder::GreedyBuilder(const Parser* parser) : parserData(parser) {}

// Método principal
Solution GreedyBuilder::buildSolution() {
    // 1. Inicializamos la solución vacía
    Solution solution(parserData);
    
    int numClients = parserData->getDimension();
    
    // Arreglo para marcar quién ya fue visitado (índice 0 no se usa)
    vector<bool> visited(numClients + 1, false);
    
    // La bodega (ID 1) no se considera un "cliente a visitar" en este conteo
    visited[1] = true; 
    
    int clientsVisited = 0;
    int totalClientsToVisit = numClients - 1; // Todos menos la bodega

    // 2. Ciclo principal: mientras queden clientes sin atender
    while (clientsVisited < totalClientsToVisit) {
        
        // Sacamos un vehículo nuevo de la bodega
        Route currentRoute(parserData->getCapacity(), parserData);
        int currentNode = 1; // El camión parte físicamente en la bodega
        
        bool addedToRoute = true;
        
        // 3. Mientras el camión pueda seguir subiendo clientes válidos
        while (addedToRoute) {
            addedToRoute = false;
            
            // Obtenemos los vecinos del nodo actual (¡ya vienen ordenados de menor a mayor distancia!)
            const auto& neighbors = parserData->getSortedNeighbors(currentNode);
            
            // Recorremos la lista de vecinos
            for (const auto& neighbor : neighbors) {
                // Si el vecino no ha sido visitado aún
                if (!visited[neighbor.id]) {
                    
                    // Intentamos agregarlo. addClient() internamente verifica si NO excede la capacidad
                    if (currentRoute.addClient(neighbor.id)) {
                        // ¡Éxito! El cliente cupo en el camión
                        visited[neighbor.id] = true;
                        clientsVisited++;
                        currentNode = neighbor.id; // El camión se mueve a este nuevo cliente
                        addedToRoute = true;
                        
                        // Rompemos el for porque ya encontramos al mejor, 
                        // ahora debemos buscar desde la nueva posición (currentNode)
                        break; 
                    }
                }
            }
            // Si termina el for y addedToRoute sigue siendo false, significa que 
            // miramos a TODOS los clientes no visitados y ninguno cupo en el espacio 
            // que le queda al camión. El ciclo interno termina.
        }
        
        // Guardamos la ruta terminada en la solución
        solution.addRoute(currentRoute);
    }

    return solution;
}

// Destructor
GreedyBuilder::~GreedyBuilder() {}