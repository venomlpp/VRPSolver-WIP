#include <iostream>
#include "Parser.h"
#include "GreedyBuilder.h"

using namespace std;

int main() {
    cout << "--- Iniciando Test del GreedyBuilder ---" << endl;
    
    // 1. Cargar el parser
    string filename = "X-n115-k10.vrp";
    Parser parser(filename);
    cout << "Instancia cargada: " << filename << endl;

    // 2. Instanciar el constructor Greedy
    GreedyBuilder greedy(&parser);
    
    // 3. Construir la solucion
    cout << "Construyendo solucion inicial (GNN)..." << endl;
    Solution solucionInicial = greedy.buildSolution();
    
    // 4. Imprimir y validar
    solucionInicial.print();
    
    cout << "\nValidacion estricta (sin subtours, sin repetir clientes, sin excesos de carga): ";
    if (solucionInicial.isValid()) {
        cout << "APROBADA [OK]" << endl;
    } else {
        cout << "REPROBADA [ERROR]" << endl;
    }

    cout << "--- Test finalizado ---" << endl;

    return 0;
}