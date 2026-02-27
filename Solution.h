#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include "Route.h"
#include "Parser.h"

class Solution {
private:
    const Parser* parserData;       // Referencia al problema global
    std::vector<Route> routes;      // El conjunto de rutas (vehículos)
    double totalCost;               // Valor de la función objetivo Z

    // Método interno para recalcular el costo total sumando las rutas
    void calculateTotalCost();

public:
    // Constructores
    Solution();
    explicit Solution(const Parser* parser);

    // Manipulación de la solución
    void addRoute(const Route& route);
    
    // El método más crítico: Valida que TODOS los clientes sean visitados exactamente una vez
    // y que ninguna ruta rompa las reglas de capacidad o subtours.
    bool isValid() const;

    // Getters
    double getTotalCost() const;
    const std::vector<Route>& getRoutes() const;

    // Equivalente a tu método "imprimir()" en Corte
    void print() const;
    
    // Destructor
    ~Solution();
};

#endif // SOLUTION_H