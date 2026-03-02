#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include "Route.h"
#include "Parser.h"

/*
 * Clase Solution
 * Descripción: Representa una solución completa para el problema CVRP.
 * Agrupa el conjunto de rutas asignadas a la flota de vehículos, 
 * gestiona el costo total (función objetivo) y valida su factibilidad global.
 */
class Solution {
private:
    const Parser* parserData;       
    std::vector<Route> routes;      
    double totalCost;               

    void calculateTotalCost();

public:
    Solution();
    explicit Solution(const Parser* parser);

    void addRoute(const Route& route);
    
    bool isValid() const;

    double getTotalCost() const;
    const std::vector<Route>& getRoutes() const;

    void print() const;
    
    ~Solution();
};

#endif // SOLUTION_H