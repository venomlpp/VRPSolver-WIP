#include "GreedyBuilder.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <list>

using namespace std;

// Estructura auxiliar para guardar y ordenar los ahorros
struct Saving {
    int i, j;
    double amount;
    
    // Sobrecarga para ordenar de mayor a menor ahorro
    bool operator<(const Saving& other) const {
        return amount > other.amount; 
    }
};

// Constructor
GreedyBuilder::GreedyBuilder(const Parser* parser) : parserData(parser) {}

// Método principal: Algoritmo de Ahorros de Clarke-Wright
Solution GreedyBuilder::buildSolution() {
    Solution solution(parserData);
    int numClients = parserData->getDimension();
    int capacity = parserData->getCapacity();

    // 1. Calcular la lista de ahorros
    vector<Saving> savings;
    
    // Los IDs de los clientes reales van del 2 al numClients (El 1 es la bodega)
    for (int i = 2; i <= numClients; ++i) {
        for (int j = i + 1; j <= numClients; ++j) {
            // Fórmula de Clarke-Wright: S_ij = d(0,i) + d(0,j) - d(i,j)
            // (Usamos 1 como índice de la bodega según tu Parser)
            double s = parserData->getDistance(1, i) + 
                       parserData->getDistance(1, j) - 
                       parserData->getDistance(i, j);
            savings.push_back({i, j, s});
        }
    }
    
    // Ordenamos los ahorros de mayor a menor
    sort(savings.begin(), savings.end());

    // 2. Inicializar rutas individuales (Bodega -> i -> Bodega)
    // Usaremos listas enlazadas (std::list) para poder fusionar O(1) en los extremos
    vector<int> routeOf(numClients + 1);
    vector<list<int>> routes(numClients + 1);
    vector<int> currentLoads(numClients + 1, 0);

    for (int i = 2; i <= numClients; ++i) {
        routeOf[i] = i;              // Al inicio, el cliente 'i' pertenece a la ruta 'i'
        routes[i].push_back(i);      // La ruta 'i' solo contiene al cliente 'i'
        currentLoads[i] = parserData->getClients()[i].getDemand();
    }

    // 3. Evaluar fusiones siguiendo el orden de mayor ahorro
    for (const auto& sav : savings) {
        int i = sav.i;
        int j = sav.j;
        
        int r1 = routeOf[i];
        int r2 = routeOf[j];

        // Regla A: Si ya están en la misma ruta, ignorar.
        if (r1 == r2) continue;

        // Regla B: Validar capacidad del vehículo
        if (currentLoads[r1] + currentLoads[r2] > capacity) continue;

        // Regla C: Para poder fusionar, 'i' y 'j' DEBEN ser extremos de sus respectivas rutas
        // (Es decir, deben estar conectados directamente a la bodega actualmente)
        bool i_is_front = (routes[r1].front() == i);
        bool i_is_back  = (routes[r1].back() == i);
        bool j_is_front = (routes[r2].front() == j);
        bool j_is_back  = (routes[r2].back() == j);

        if ((i_is_front || i_is_back) && (j_is_front || j_is_back)) {
            
            // Alinear las listas para que 'i' quede al final de r1 y 'j' quede al inicio de r2
            if (i_is_front) routes[r1].reverse();
            if (j_is_back)  routes[r2].reverse();

            // Actualizar el identificador de ruta (routeOf) para todos los clientes absorbidos
            for (int client : routes[r2]) {
                routeOf[client] = r1;
            }

            // Fusionar: Enganchar r2 al final de r1
            routes[r1].splice(routes[r1].end(), routes[r2]);
            
            // Actualizar la carga acumulada
            currentLoads[r1] += currentLoads[r2];
            currentLoads[r2] = 0; // La ruta r2 queda vacía y obsoleta
        }
    }

    // 4. Transformar nuestras listas temporales en objetos 'Route' oficiales
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

// Destructor
GreedyBuilder::~GreedyBuilder() {}