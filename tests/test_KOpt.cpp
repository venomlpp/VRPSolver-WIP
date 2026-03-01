#include <iostream>
#include "Parser.h"
#include "GreedyBuilder.h"
#include "KOpt.h"

using namespace std;

int main() {
    cout << "--- Iniciando Test de Heuristica 3-OPT ---" << endl;
    
    string filename = "X-n115-k10.vrp";
    Parser parser(filename);
    
    // 1. Fase de Construccion (GNN)
    GreedyBuilder greedy(&parser);
    Solution solucionInicial = greedy.buildSolution();
    cout << "Costo Inicial (Greedy): " << solucionInicial.getTotalCost() << endl;

    // 2. Fase de Mejora (3-OPT)
    cout << "Ejecutando Optimizacion 3-OPT..." << endl;
    KOpt optimizador(&parser);
    Solution solucionMejorada = optimizador.optimize(solucionInicial);
    
    // 3. Imprimir Resultados
    solucionMejorada.print();
    
    cout << "\n--- Resumen de Mejora ---" << endl;
    cout << "Costo Inicial: " << solucionInicial.getTotalCost() << endl;
    cout << "Costo Optimizado: " << solucionMejorada.getTotalCost() << endl;
    cout << "Ahorro total: " << solucionInicial.getTotalCost() - solucionMejorada.getTotalCost() << endl;

    cout << "\nValidacion estricta (sin subtours/excesos): ";
    if (solucionMejorada.isValid()) {
        cout << "APROBADA [OK]" << endl;
    } else {
        cout << "REPROBADA [ERROR]" << endl;
    }

    return 0;
}