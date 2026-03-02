#include "Solution.h"
#include <iostream>
#include <set>

using namespace std;

/*
 * Descripción: Constructor por defecto. Inicializa una solución vacía sin datos.
 * Entrada: Ninguna.
 * Salida: Instancia de Solution con puntero nulo y costo 0.
 */
Solution::Solution() : parserData(nullptr), totalCost(0.0) {}

/*
 * Descripción: Constructor parametrizado. Inicializa una solución vacía ligada a la instancia.
 * Entrada: Puntero constante a los datos parseados de la instancia.
 * Salida: Instancia de Solution lista para almacenar rutas.
 */
Solution::Solution(const Parser* parser) : parserData(parser), totalCost(0.0) {}

/*
 * Descripción: Recalcula el costo total sumando la función objetivo de cada ruta en la flota.
 * Entrada: Ninguna.
 * Salida: Ninguna (actualiza el atributo totalCost internamente).
 */
void Solution::calculateTotalCost() {
    totalCost = 0.0;
    for (const auto& route : routes) {
        totalCost += route.getTotalCost();
    }
}

/*
 * Descripción: Incorpora una ruta a la solución y actualiza el costo global.
 * Entrada: Objeto Route evaluado y construido.
 * Salida: Ninguna.
 */
void Solution::addRoute(const Route& route) {
    routes.push_back(route);
    calculateTotalCost(); 
}

/*
 * Descripción: Valida la factibilidad estructural de toda la solución. Verifica que todas
 * las rutas sean válidas individualmente y que cada cliente sea visitado exactamente una vez.
 * Entrada: Ninguna.
 * Salida: Booleano que indica si la solución es estrictamente factible.
 */
bool Solution::isValid() const {
    if (!parserData) return false;

    int numClients = parserData->getDimension();
    vector<int> visitCount(numClients + 1, 0);

    for (const auto& route : routes) {
        if (!route.isValid()) {
            return false;
        }

        const auto& path = route.getPath();
        for (int nodeId : path) {
            if (nodeId != 1) {
                visitCount[nodeId]++;
            }
        }
    }

    for (int i = 2; i <= numClients; ++i) {
        if (visitCount[i] != 1) {
            return false; 
        }
    }

    return true;
}

/*
 * Descripción: Retorna el costo total global (Z) de la solución.
 * Entrada: Ninguna.
 * Salida: Decimal con el valor de la función objetivo.
 */
double Solution::getTotalCost() const { return totalCost; }

/*
 * Descripción: Retorna el conjunto de rutas asignadas a los vehículos.
 * Entrada: Ninguna.
 * Salida: Referencia constante al vector de objetos Route.
 */
const std::vector<Route>& Solution::getRoutes() const { return routes; }

/*
 * Descripción: Despliega por consola el reporte detallado de la solución (Z, K, cargas, costos y secuencias).
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
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

/*
 * Descripción: Destructor de la clase.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
Solution::~Solution() {}