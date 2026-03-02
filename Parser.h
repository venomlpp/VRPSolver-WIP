#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <vector>
#include "Client.h"

/*
 * Estructura Neighbor
 * Descripción: Estructura auxiliar para la lista de adyacencia ordenada.
 * Almacena el ID de un nodo vecino y su distancia.
 */
struct Neighbor {
    int id;
    int distance;
    
    bool operator<(const Neighbor& other) const {
        return distance < other.distance;
    }
};

/*
 * Clase Parser
 * Descripción: Encargada de leer y procesar los archivos de instancias CVRP.
 * Calcula la matriz de distancias y construye una lista de adyacencia ordenada.
 */
class Parser {
private:
    std::string filename;
    int dimension; 
    int capacity;  
    
    std::vector<Client> clients;
    std::vector<std::vector<int>> distanceMatrix; 
    std::vector<std::vector<Neighbor>> sortedAdjacencyList;

    void loadData();
    void calculateDistanceMatrix();
    void buildSortedAdjacencyList();

public:
    explicit Parser(const std::string& filename);

    int getDimension() const;
    int getCapacity() const;

    const std::vector<Client>& getClients() const;
    const std::vector<std::vector<int>>& getDistanceMatrix() const;
    const std::vector<Neighbor>& getSortedNeighbors(int nodeId) const;
    
    int getDistance(int fromId, int toId) const;

    ~Parser();
};

#endif // PARSER_H