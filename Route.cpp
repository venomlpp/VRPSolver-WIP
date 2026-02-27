#include "Route.h"
#include <iostream>

// Constructor
Route::Route(int maxCapacity, const Parser* parser) 
    : currentLoad(0), totalCost(0.0), maxCapacity(maxCapacity), parserData(parser) {
    
    // Toda ruta debe iniciar y terminar obligatoriamente en la bodega (ID 1)
    path.push_back(1); // Inicio
    path.push_back(1); // Fin
}

// Recalcula el costo total iterando sobre el path actual
// (Este método se puede optimizar a O(1) más adelante para inserciones/borrados específicos)
void Route::updateMetrics() {
    totalCost = 0.0;
    // Sumamos la distancia de cada segmento: (i) -> (i+1)
    for (size_t i = 0; i < path.size() - 1; ++i) {
        totalCost += parserData->getDistance(path[i], path[i + 1]);
    }
}

// Intenta agregar un cliente al FINAL de la ruta (justo antes del regreso a la bodega)
bool Route::addClient(int clientId) {
    // 1. Verificamos que el cliente exista y obtenemos su demanda
    int demand = parserData->getClients()[clientId].getDemand();

    // 2. Validamos la restricción de capacidad
    if (currentLoad + demand > maxCapacity) {
        return false; // El vehículo no tiene capacidad suficiente
    }

    // 3. Insertamos el cliente justo antes del último elemento (que siempre es el ID 1, la bodega)
    path.insert(path.end() - 1, clientId);
    
    // 4. Actualizamos la carga y los costos
    currentLoad += demand;
    updateMetrics(); 

    return true;
}

// Inserta un cliente en una posición específica (útil para heurísticas futuras)
void Route::insertClientAt(int index, int clientId) {
    // Protegemos los extremos (no se puede insertar en índice 0 ni al final sobreescribiendo el ID 1)
    if (index <= 0 || index >= static_cast<int>(path.size())) {
        std::cerr << "Error: Indice de insercion invalido en Route.\n";
        return;
    }
    
    int demand = parserData->getClients()[clientId].getDemand();
    if (currentLoad + demand <= maxCapacity) {
        path.insert(path.begin() + index, clientId);
        currentLoad += demand;
        updateMetrics();
    }
}

// Remueve un cliente por su ID (busca la primera aparición)
void Route::removeClient(int clientId) {
    for (auto it = path.begin() + 1; it != path.end() - 1; ++it) {
        if (*it == clientId) {
            int demand = parserData->getClients()[clientId].getDemand();
            currentLoad -= demand;
            path.erase(it);
            updateMetrics();
            return; // Solo remueve uno y termina
        }
    }
}

// Validación de correctitud (Exigida por el profesor para evitar subtours)
bool Route::isValid() const {
    if (path.empty() || path.front() != 1 || path.back() != 1) return false;
    if (currentLoad > maxCapacity) return false;
    return true;
}

// Getters
int Route::getCurrentLoad() const { return currentLoad; }
double Route::getTotalCost() const { return totalCost; }
const std::vector<int>& Route::getPath() const { return path; }

// Destructor
Route::~Route() {}