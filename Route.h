#ifndef ROUTE_H
#define ROUTE_H

#include <vector>
#include "Parser.h" 

/*
 * Clase Route
 * Descripción: Representa la ruta de un solo vehículo en el problema CVRP.
 * Mantiene la secuencia de clientes visitados, la carga acumulada y el costo total.
 */
class Route {
private:
    std::vector<int> path; 
    int currentLoad;
    double totalCost;
    int maxCapacity;

    const Parser* parserData; 

    void updateMetrics();

public:
    Route(int maxCapacity, const Parser* parser);

    bool addClient(int clientId); 
    void insertClientAt(int index, int clientId);
    void removeClient(int clientId);

    bool isValid() const; 
    int getCurrentLoad() const;
    double getTotalCost() const;
    const std::vector<int>& getPath() const;

    ~Route();
};

#endif // ROUTE_H