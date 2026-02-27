#include <iostream>
#include "Parser.h"

using namespace std;

int main() {
    cout << "--- Iniciando Test del Parser (con Lista Adyacencia) ---" << endl;
    
    // Usamos la instancia pequeña de prueba
    string filename = "toy.vrp";
    Parser parser(filename);

    cout << "Archivo leido: " << filename << endl;
    cout << "Dimension total: " << parser.getDimension() << endl;
    cout << "Capacidad vehicular (Q): " << parser.getCapacity() << endl;
    cout << "---------------------------------" << endl;

    // Vamos a imprimir la lista de vecinos ordenados para la Bodega (Nodo 1)
    cout << "Vecinos ordenados por distancia desde la BODEGA (Nodo 1):" << endl;
    
    const auto& vecinosBodega = parser.getSortedNeighbors(1);
    
    for (size_t i = 0; i < vecinosBodega.size(); ++i) {
        cout << i + 1 << "° mas cercano -> Cliente ID: " << vecinosBodega[i].id 
             << " | Distancia: " << vecinosBodega[i].distance 
             << " | Demanda: " << parser.getClients()[vecinosBodega[i].id].getDemand() << endl;
    }

    cout << "---------------------------------" << endl;

    // Probemos con el Cliente 3
    cout << "Vecinos ordenados por distancia desde el Cliente 3:" << endl;
    const auto& vecinosCliente3 = parser.getSortedNeighbors(3);
    
    for (size_t i = 0; i < vecinosCliente3.size(); ++i) {
        cout << i + 1 << "° mas cercano -> Cliente ID: " << vecinosCliente3[i].id 
             << " | Distancia: " << vecinosCliente3[i].distance << endl;
    }

    cout << "--- Test superado exitosamente ---" << endl;

    return 0;
}