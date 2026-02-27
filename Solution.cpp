#include "Solution.h"
#include <iostream>
#include <set>

using namespace std;

// Constructor por defecto (para cuando necesitamos instanciar antes de tener datos)
Solution::Solution() : parserData(nullptr), totalCost(0.0) {}

// Constructor parametrizado (el que más usaremos)
Solution::Solution(const Parser* parser) : parserData(parser), totalCost(0.0) {}

// Recalcula el costo sumando el de cada ruta individual
void Solution::calculateTotalCost() {
    totalCost = 0.0;
    for (const auto& route : routes) {
        totalCost += route.getTotalCost();
    }
}

// Agrega una nueva ruta a la flota
void Solution::addRoute(const Route& route) {
    routes.push_back(route);
    calculateTotalCost(); // Actualiza el costo Z global
}

// Validación estricta según rúbrica:
// 1. Cada ruta debe ser válida (inicio/fin en bodega, no exceder capacidad).
// 2. Todos los clientes (del 2 al N) deben ser visitados exactamente una vez.
bool Solution::isValid() const {
    if (!parserData) return false;

    int numClients = parserData->getDimension();
    // Arreglo para contar visitas (índice 0 no se usa, el 1 es la bodega)
    vector<int> visitCount(numClients + 1, 0);

    for (const auto& route : routes) {
        // Validación 1: Verificar reglas de capacidad y extremos (bodega)
        if (!route.isValid()) {
            return false;
        }

        const auto& path = route.getPath();
        // Contar visitas de clientes (ignoramos la bodega, índices 1)
        for (int nodeId : path) {
            if (nodeId != 1) {
                visitCount[nodeId]++;
            }
        }
    }

    // Validación 2: Verificar que todos los clientes (2 al N) tengan exactamente 1 visita
    for (int i = 2; i <= numClients; ++i) {
        if (visitCount[i] != 1) {
            return false; // Falta un cliente o se visitó más de una vez
        }
    }

    return true;
}

// Getters
double Solution::getTotalCost() const { return totalCost; }
const std::vector<Route>& Solution::getRoutes() const { return routes; }

// Imprime el estado completo de la solución
void Solution::print() const {
    cout << "=== Solucion CVRP ===" << endl;
    cout << "Costo Total (Z): " << totalCost << endl;
    cout << "Numero de Vehiculos Utilizados: " << routes.size() << endl;
    
    for (size_t i = 0; i < routes.size(); ++i) {
        cout << "Vehiculo " << i + 1 << " | Carga: " << routes[i].getCurrentLoad() 
             << " | Costo: " << routes[i].getTotalCost() << " | Ruta: [ ";
        
        for (int nodeId : routes[i].getPath()) {
            cout << nodeId << " ";
        }
        cout << "]" << endl;
    }
    cout << "=====================" << endl;
}

// Destructor
Solution::~Solution() {}