#include <iostream>
#include "Parser.h"
#include "CbcSolver.h"

using namespace std;

int main() {
    cout << "--- Iniciando Test del Solver CBC (Fase 4) ---" << endl;
    
    string filename = "A-n62-k8.vrp"; 
    Parser parser(filename);
    
    CbcSolver solver(&parser);
    
    // Le decimos a CBC que encuentre el óptimo exacto para 5 vehículos
    Solution solucionMIP = solver.solve(5);
    
    cout << "\n=== RESULTADO FINAL CBC ===" << endl;
    solucionMIP.print();
    
    cout << "\nValidacion estricta (sin subtours/excesos): ";
    if (solucionMIP.isValid()) {
        cout << "APROBADA [OK]" << endl;
    } else {
        cout << "REPROBADA [ERROR]" << endl;
    }

    return 0;
}