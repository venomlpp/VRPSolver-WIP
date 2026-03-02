#include "GreedyBuilder.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <list>

using namespace std;

/*
 * Estructura Saving
 * Descripción: Estructura auxiliar que almacena el ahorro de unir dos nodos i, j, 
 * y permite ordenar la lista de ahorros de mayor a menor.
 */
struct Saving {
    int i, j;
    double amount;
    
    bool operator<(const Saving& other) const {
        return amount > other.amount; 
    }
};

/*
 * Descripción: Constructor del generador de soluciones iniciales.
 * Entrada: Puntero constante a los datos parseados de la instancia.
 * Salida: Instancia inicializada.
 */
GreedyBuilder::GreedyBuilder(const Parser* parser) : parserData(parser) {}

/*
 * Descripción: Ejecuta el algoritmo de Ahorros de Clarke-Wright. Calcula los ahorros
 * de fusionar rutas individuales y las une iterativamente respetando restricciones.
 * Entrada: Ninguna.
 * Salida: Objeto Solution con la solución inicial factible.
 */
Solution GreedyBuilder::buildSolution() {
    Solution solution(parserData);
    int numClients = parserData->getDimension();
    int capacity = parserData->getCapacity();

    // ── 1. Calcular la lista de ahorros ───────────────────────────
    vector<Saving> savings;
    
    for (int i = 2; i <= numClients; ++i) {
        for (int j = i + 1; j <= numClients; ++j) {
            double s = parserData->getDistance(1, i) + 
                       parserData->getDistance(1, j) - 
                       parserData->getDistance(i, j);
            savings.push_back({i, j, s});
        }
    }
    
    sort(savings.begin(), savings.end());

    // ── 2. Inicializar rutas individuales ─────────────────────────
    vector<int> routeOf(numClients + 1);
    vector<list<int>> routes(numClients + 1);
    vector<int> currentLoads(numClients + 1, 0);

    for (int i = 2; i <= numClients; ++i) {
        routeOf[i] = i;              
        routes[i].push_back(i);      
        currentLoads[i] = parserData->getClients()[i].getDemand();
    }

    // ── 3. Evaluar fusiones según orden de ahorro ─────────────────
    for (const auto& sav : savings) {
        int i = sav.i;
        int j = sav.j;
        
        int r1 = routeOf[i];
        int r2 = routeOf[j];

        if (r1 == r2) continue;

        if (currentLoads[r1] + currentLoads[r2] > capacity) continue;

        // Para poder fusionar, 'i' y 'j' deben ser extremos de sus respectivas rutas
        bool i_is_front = (routes[r1].front() == i);
        bool i_is_back  = (routes[r1].back() == i);
        bool j_is_front = (routes[r2].front() == j);
        bool j_is_back  = (routes[r2].back() == j);

        if ((i_is_front || i_is_back) && (j_is_front || j_is_back)) {
            
            // Alinear las listas para que 'i' quede al final de r1 y 'j' quede al inicio de r2
            if (i_is_front) routes[r1].reverse();
            if (j_is_back)  routes[r2].reverse();

            for (int client : routes[r2]) {
                routeOf[client] = r1;
            }

            routes[r1].splice(routes[r1].end(), routes[r2]);
            
            currentLoads[r1] += currentLoads[r2];
            currentLoads[r2] = 0; 
        }
    }

    // ── 4. Construir solución formal ──────────────────────────────
    for (int i = 2; i <= numClients; ++i) {
        if (!routes[i].empty()) {
            Route finalRoute(capacity, parserData);
            
            for (int clientID : routes[i]) {
                finalRoute.addClient(clientID);
            }
            solution.addRoute(finalRoute);
        }
    }

    return solution;
}

/*
 * Descripción: Destructor de la clase.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
GreedyBuilder::~GreedyBuilder() {}