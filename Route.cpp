#include "Route.h"
#include <iostream>

/*
 * Descripción: Constructor de la clase Route. Inicializa la ruta comenzando 
 * y terminando en la bodega (ID 1).
 * Entrada: Capacidad máxima del vehículo, puntero al parser de datos.
 * Salida: Instancia de Route inicializada.
 */
Route::Route(int maxCapacity, const Parser* parser) 
    : currentLoad(0), totalCost(0.0), maxCapacity(maxCapacity), parserData(parser) {
    path.push_back(1); 
    path.push_back(1); 
}

/*
 * Descripción: Recalcula el costo total de la ruta iterando sobre todos sus segmentos.
 * Entrada: Ninguna.
 * Salida: Ninguna (actualiza el atributo totalCost internamente).
 */
void Route::updateMetrics() {
    totalCost = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        totalCost += parserData->getDistance(path[i], path[i + 1]);
    }
}

/*
 * Descripción: Intenta agregar un cliente al final de la ruta, justo antes del regreso a la bodega.
 * Entrada: ID del cliente a agregar.
 * Salida: Booleano indicando si la inserción fue exitosa (true) o si violaría la capacidad (false).
 */
bool Route::addClient(int clientId) {
    int demand = parserData->getClients()[clientId].getDemand();

    if (currentLoad + demand > maxCapacity) {
        return false; 
    }

    path.insert(path.end() - 1, clientId);
    currentLoad += demand;
    updateMetrics();

    return true;
}

/*
 * Descripción: Inserta un cliente en una posición específica de la ruta.
 * Entrada: Índice de inserción, ID del cliente a insertar.
 * Salida: Ninguna.
 */
void Route::insertClientAt(int index, int clientId) {
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

/*
 * Descripción: Remueve la primera aparición de un cliente específico dentro de la ruta.
 * Entrada: ID del cliente a remover.
 * Salida: Ninguna.
 */
void Route::removeClient(int clientId) {
    for (auto it = path.begin() + 1; it != path.end() - 1; ++it) {
        if (*it == clientId) {
            int demand = parserData->getClients()[clientId].getDemand();
            currentLoad -= demand;
            path.erase(it);
            updateMetrics();
            return; 
        }
    }
}

/*
 * Descripción: Valida la correctitud estructural de la ruta.
 * Entrada: Ninguna.
 * Salida: Booleano indicando si la ruta inicia/termina en bodega y no excede la capacidad.
 */
bool Route::isValid() const {
    if (path.empty() || path.front() != 1 || path.back() != 1) return false;
    if (currentLoad > maxCapacity) return false;
    return true;
}

/*
 * Descripción: Retorna la carga actual acumulada en la ruta.
 * Entrada: Ninguna.
 * Salida: Entero con la carga actual.
 */
int Route::getCurrentLoad() const { return currentLoad; }

/*
 * Descripción: Retorna el costo total (distancia) de la ruta.
 * Entrada: Ninguna.
 * Salida: Decimal con el costo total.
 */
double Route::getTotalCost() const { return totalCost; }

/*
 * Descripción: Retorna la secuencia completa de clientes de la ruta.
 * Entrada: Ninguna.
 * Salida: Referencia constante al vector que representa la ruta.
 */
const std::vector<int>& Route::getPath() const { return path; }

/*
 * Descripción: Destructor de la clase.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
Route::~Route() {}