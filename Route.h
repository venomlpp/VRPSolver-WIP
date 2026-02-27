#ifndef ROUTE_H
#define ROUTE_H

#include <vector>
#include "Parser.h" // Necesitamos la matriz de distancias para calcular costos

class Route {
private:
    std::vector<int> path; // Secuencia de IDs de clientes (ej: 1 -> 4 -> 2 -> 1)
    int currentLoad;
    double totalCost;
    int maxCapacity;

    // Referencia al parser (o a la matriz) para no duplicar datos
    const Parser* parserData; 

    // Actualiza el costo y la carga internamente al modificar la ruta
    void updateMetrics();

public:
    // Constructor
    Route(int maxCapacity, const Parser* parser);

    // Métodos de manipulación de la ruta
    bool addClient(int clientId); // Retorna falso si excede la capacidad
    void insertClientAt(int index, int clientId);
    void removeClient(int clientId);

    // Métodos de validación y consulta
    bool isValid() const; // Verifica que empiece/termine en 1 y no exceda capacidad
    int getCurrentLoad() const;
    double getTotalCost() const;
    const std::vector<int>& getPath() const;

    // Destructor
    ~Route();
};

#endif // ROUTE_H