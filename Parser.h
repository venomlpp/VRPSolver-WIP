#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <vector>
#include "Client.h"

// Estructura auxiliar para la lista de adyacencia ordenada
struct Neighbor {
    int id;
    int distance;
    
    // Sobrecarga del operador < para que std::sort sepa cómo ordenarlos de menor a mayor
    bool operator<(const Neighbor& other) const {
        return distance < other.distance;
    }
};

class Parser {
private:
    std::string filename;
    int dimension; // Número total de nodos (bodega + clientes)
    int capacity;  // Capacidad Q de los vehículos
    
    std::vector<Client> clients;
    std::vector<std::vector<int>> distanceMatrix; 

    // NUEVO: Lista de adyacencia donde la posición 'u' contiene a todos los vecinos ordenados
    std::vector<std::vector<Neighbor>> sortedAdjacencyList;

    // Métodos privados auxiliares
    void loadData();
    void calculateDistanceMatrix();
    // NUEVO: Método para construir y ordenar la lista de adyacencia
    void buildSortedAdjacencyList();

public:
    // Constructor
    explicit Parser(const std::string& filename);

    // Getters de datos globales
    int getDimension() const;
    int getCapacity() const;

    // Getters de estructuras 
    const std::vector<Client>& getClients() const;
    const std::vector<std::vector<int>>& getDistanceMatrix() const;
    
    // NUEVO: Getter para acceder a los vecinos ordenados de un nodo específico en O(1)
    const std::vector<Neighbor>& getSortedNeighbors(int nodeId) const;
    
    // Método auxiliar para obtener la distancia entre dos nodos en O(1)
    int getDistance(int fromId, int toId) const;

    // Destructor
    ~Parser();
};

#endif // PARSER_H