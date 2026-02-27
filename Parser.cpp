#include "Parser.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm> // Para std::sort

using namespace std;

// Constructor: inicializa variables y dispara la lectura
Parser::Parser(const string& filename) : filename(filename), dimension(0), capacity(0) {
    loadData();
    calculateDistanceMatrix();
    buildSortedAdjacencyList(); // NUEVO: Pre-calculamos los vecinos ordenados
}

void Parser::loadData() {
    ifstream vrp(filename);
    if (!vrp.is_open()) {
        cerr << "Error opening file: " << filename << "\n";
        exit(1);
    }

    int idx;
    double x, y;
    int q;
    string line;
    
    vector<pair<double, double>> coords;
    vector<int> demand;
    int depot_index_basura = -1; 

    while (getline(vrp, line)) {
        if (line.rfind("DIMENSION", 0) == 0) {
            dimension = stoi(line.substr(line.find(":") + 1));
            demand.resize(dimension + 1, 0);
            coords.resize(dimension + 1, {0.0, 0.0});
            clients.resize(dimension + 1); 
        } else if (line.rfind("CAPACITY", 0) == 0) {
            capacity = stoi(line.substr(line.find(":") + 1));
        } else if (line.rfind("NODE_COORD_SECTION", 0) == 0) {
            for (int i = 0; i < dimension; ++i) {
                vrp >> idx >> x >> y;
                coords[idx] = {x, y};
            }
        } else if (line.rfind("DEMAND_SECTION", 0) == 0) {
            for (int i = 0; i < dimension; ++i) {
                vrp >> idx >> q;
                demand[idx] = q;
            }
        } else if (line.rfind("DEPOT_SECTION", 0) == 0) {
            vrp >> depot_index_basura; 
        }
    }
    vrp.close();

    for (int i = 1; i <= dimension; ++i) {
        clients[i] = Client(i, coords[i].first, coords[i].second, demand[i]);
    }
}

void Parser::calculateDistanceMatrix() {
    distanceMatrix.resize(dimension + 1, vector<int>(dimension + 1, 0));

    for (int i = 1; i <= dimension; ++i) {
        for (int j = 1; j <= dimension; ++j) {
            double dx = clients[i].getX() - clients[j].getX();
            double dy = clients[i].getY() - clients[j].getY();
            distanceMatrix[i][j] = (int)(sqrt(dx * dx + dy * dy));
        }
    }
}

// NUEVO: Construcción de la lista de adyacencia ordenada
void Parser::buildSortedAdjacencyList() {
    // Redimensionamos para tener un vector por cada nodo (índice 0 no se usa)
    sortedAdjacencyList.resize(dimension + 1);

    for (int i = 1; i <= dimension; ++i) {
        // Para cada nodo i, agregamos a todos los demás nodos j
        for (int j = 1; j <= dimension; ++j) {
            if (i != j) { // No agregamos la distancia hacia sí mismo
                sortedAdjacencyList[i].push_back({j, distanceMatrix[i][j]});
            }
        }
        // Ordenamos los vecinos del nodo i de menor a mayor distancia
        std::sort(sortedAdjacencyList[i].begin(), sortedAdjacencyList[i].end());
    }
}

int Parser::getDimension() const { return dimension; }
int Parser::getCapacity() const { return capacity; }
const vector<Client>& Parser::getClients() const { return clients; }
const vector<vector<int>>& Parser::getDistanceMatrix() const { return distanceMatrix; }
int Parser::getDistance(int fromId, int toId) const { return distanceMatrix[fromId][toId]; }

// NUEVO: Retorna la lista de vecinos ordenados de un nodo
const vector<Neighbor>& Parser::getSortedNeighbors(int nodeId) const {
    return sortedAdjacencyList[nodeId];
}

Parser::~Parser() {}