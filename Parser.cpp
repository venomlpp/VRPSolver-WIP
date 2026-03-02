#include "Parser.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm> 

using namespace std;

/*
 * Descripción: Constructor de la clase Parser. Inicializa las variables,
 * lee el archivo y precalcula las distancias y vecinos.
 * Entrada: Ruta del archivo de la instancia (string).
 * Salida: Instancia inicializada.
 */
Parser::Parser(const string& filename) : filename(filename), dimension(0), capacity(0) {
    loadData();
    calculateDistanceMatrix();
    buildSortedAdjacencyList(); 
}

/*
 * Descripción: Lee los datos del archivo .vrp (dimensión, capacidad, coordenadas y demandas).
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
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

/*
 * Descripción: Calcula la matriz de distancias entre todos los clientes.
 * Utiliza std::round para cumplir con la norma EUC_2D oficial del TSPLIB.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
void Parser::calculateDistanceMatrix() {
    distanceMatrix.resize(dimension + 1, vector<int>(dimension + 1, 0));

    for (int i = 1; i <= dimension; ++i) {
        for (int j = 1; j <= dimension; ++j) {
            double dx = clients[i].getX() - clients[j].getX();
            double dy = clients[i].getY() - clients[j].getY();
            
            distanceMatrix[i][j] = static_cast<int>(std::round(std::sqrt(dx * dx + dy * dy)));
        }
    }
}

/*
 * Descripción: Construye una lista de adyacencia donde cada nodo tiene a sus
 * vecinos ordenados de menor a mayor distancia.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
void Parser::buildSortedAdjacencyList() {
    sortedAdjacencyList.resize(dimension + 1);

    for (int i = 1; i <= dimension; ++i) {
        for (int j = 1; j <= dimension; ++j) {
            if (i != j) { 
                sortedAdjacencyList[i].push_back({j, distanceMatrix[i][j]});
            }
        }
        std::sort(sortedAdjacencyList[i].begin(), sortedAdjacencyList[i].end());
    }
}

/*
 * Descripción: Retorna la dimensión del problema (número de clientes + bodega).
 * Entrada: Ninguna.
 * Salida: Entero con la dimensión.
 */
int Parser::getDimension() const { return dimension; }

/*
 * Descripción: Retorna la capacidad máxima de los vehículos.
 * Entrada: Ninguna.
 * Salida: Entero con la capacidad.
 */
int Parser::getCapacity() const { return capacity; }

/*
 * Descripción: Retorna el vector de clientes.
 * Entrada: Ninguna.
 * Salida: Referencia constante al vector de clientes.
 */
const vector<Client>& Parser::getClients() const { return clients; }

/*
 * Descripción: Retorna la matriz de distancias precalculada.
 * Entrada: Ninguna.
 * Salida: Referencia constante a la matriz 2D de distancias.
 */
const vector<vector<int>>& Parser::getDistanceMatrix() const { return distanceMatrix; }

/*
 * Descripción: Retorna la distancia exacta entre dos nodos.
 * Entrada: ID del nodo origen, ID del nodo destino.
 * Salida: Entero con la distancia.
 */
int Parser::getDistance(int fromId, int toId) const { return distanceMatrix[fromId][toId]; }

/*
 * Descripción: Retorna la lista de vecinos de un nodo, ordenados por distancia.
 * Entrada: ID del nodo a consultar.
 * Salida: Referencia constante al vector de vecinos ordenados.
 */
const vector<Neighbor>& Parser::getSortedNeighbors(int nodeId) const {
    return sortedAdjacencyList[nodeId];
}

/*
 * Descripción: Destructor de la clase.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
Parser::~Parser() {}