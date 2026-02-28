#include <iostream>
#include <string>
#include "Parser.h"
#include "GreedyBuilder.h"
#include "KOpt.h"
#include "BranchAndBound.h"

using namespace std;

int main() {
    cout << "--- Iniciando Test del Solver Branch & Bound (CLP + DFJ) ---" << endl;
    
    // NOTA: Prueba primero con "toy.vrp". Cuando estés seguro de que compila 
    // y corre sin segmentation faults, cámbialo a "X-n115-k10.vrp"
    string filename = "A-n62-k8.vrp"; // Cambia a tu instancia de prueba aquí
    
    cout << "1. Cargando instancia: " << filename << "..." << endl;
    Parser parser(filename);
    
    // --- FASE 1: Construcción ---
    cout << "2. Ejecutando Clarke-Wright..." << endl;
    GreedyBuilder greedy(&parser);
    Solution solucionGreedy = greedy.buildSolution();
    cout << "   -> Costo CW: " << solucionGreedy.getTotalCost() << endl;

    // --- FASE 2: Mejora Heurística (Upper Bound Inicial) ---
    cout << "3. Ejecutando 3-OPT para establecer Upper Bound global..." << endl;
    KOpt optimizador(&parser);
    Solution solucionKOpt = optimizador.optimize(solucionGreedy);
    cout << "   -> Costo 3-OPT (Upper Bound): " << solucionKOpt.getTotalCost() << endl;

    // --- FASE 3: Branch & Bound Continuo ---
    cout << "\n4. Iniciando Branch & Bound con modelo COIN-OR CLP..." << endl;
    cout << "   (Esto puede tomar tiempo dependiendo de la cantidad de nodos)" << endl;
    
    BranchAndBound bbSolver(&parser, solucionKOpt);
    Solution solucionFinal = bbSolver.solveBestFirst();

    // --- RESULTADOS FINALES ---
    cout << "\n=== RESULTADO FINAL BRANCH & BOUND ===" << endl;
    solucionFinal.print();
    
    cout << "\nValidacion estricta (sin subtours/excesos): ";
    if (solucionFinal.isValid()) {
        cout << "APROBADA [OK]" << endl;
    } else {
        cout << "REPROBADA [ERROR]" << endl;
    }

    cout << "--- Test finalizado ---" << endl;

    return 0;
}