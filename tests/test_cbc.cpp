// test_cbc.cpp
// Prueba integral de la clase CbcSolver.
// Verifica que CBC mejore la solución VNS inicial, reporte GAP,
// y entregue una solución válida sin subtours ni violaciones de capacidad.
//
// ADVERTENCIA: Cada test ejecuta CBC con límite de TIME_LIMIT_SECONDS.
// El tiempo total de ejecución puede ser de varios minutos.
// Para pruebas rápidas, pasar una instancia pequeña como argumento.

#include <iostream>
#include <string>
#include <cassert>
#include "Parser.h"
#include "GreedyBuilder.h"
#include "VNS.h"
#include "CbcSolver.h"
#include "Solution.h"

using namespace std;

// Límite de tiempo para CBC.
// Usar el mismo valor que TIME_LIMIT_SECONDS en test_BB
// para una comparación justa entre B&B y CBC.
const double TIME_LIMIT_SECONDS = 120.0;

// ─────────────────────────────────────────────────────────────
// Imprime una solución con detalle de rutas
// ─────────────────────────────────────────────────────────────
void printSolution(const Solution& sol, const string& label) {
    cout << "\n--- " << label << " ---" << endl;
    cout << "Costo total : " << sol.getTotalCost() << endl;
    cout << "Vehiculos   : " << sol.getRoutes().size() << endl;
    for (size_t i = 0; i < sol.getRoutes().size(); i++) {
        const auto& route = sol.getRoutes()[i];
        cout << "  Vehiculo " << i + 1
             << " | Carga: " << route.getCurrentLoad()
             << " | Costo: " << route.getTotalCost()
             << " | Ruta: [ ";
        for (int node : route.getPath()) cout << node << " ";
        cout << "]" << endl;
    }
}

// ─────────────────────────────────────────────────────────────
// Deduce numVehicles desde la solución VNS (cantidad de rutas)
// ─────────────────────────────────────────────────────────────
int deduceVehicles(const Solution& sol) {
    return static_cast<int>(sol.getRoutes().size());
}

// ─────────────────────────────────────────────────────────────
// Test 1: CBC produce una solución válida
// ─────────────────────────────────────────────────────────────
void testCbcValidity(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 1: CBC produce solucion valida" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    VNS vns(&parser);
    Solution vnsSol = vns.optimize(builder.buildSolution());

    CbcSolver cbc(&parser);
    Solution cbcSol = cbc.solve(vnsSol, TIME_LIMIT_SECONDS);

    assert(cbcSol.isValid() &&
           "ERROR: CBC produjo una solucion invalida (subtour o violacion de capacidad).");

    printSolution(cbcSol, "Solucion CBC");
    cout << "PASS: Solucion CBC es valida." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 2: CBC no empeora el warm start VNS
// ─────────────────────────────────────────────────────────────
void testCbcNotWorseVNS(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 2: CBC no empeora warm start VNS" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    VNS vns(&parser);
    Solution vnsSol = vns.optimize(builder.buildSolution());

    cout << "Costo VNS (warm start): " << vnsSol.getTotalCost() << endl;

    CbcSolver cbc(&parser);
    Solution cbcSol = cbc.solve(vnsSol, TIME_LIMIT_SECONDS);

    cout << "Costo CBC (resultado) : " << cbcSol.getTotalCost() << endl;

    assert(cbcSol.getTotalCost() <= vnsSol.getTotalCost() &&
           "ERROR: CBC devolvio un costo peor que su warm start VNS.");

    assert(cbcSol.isValid() &&
           "ERROR: CBC produjo una solucion invalida.");

    cout << "PASS: CBC no empeoro el warm start VNS." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 3: Pipeline completo CW -> VNS -> CBC
// ─────────────────────────────────────────────────────────────
void testFullPipeline(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 3: Pipeline CW -> VNS -> CBC" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    Solution cwSol = builder.buildSolution();
    assert(cwSol.isValid() && "ERROR: Clarke-Wright produjo solucion invalida.");

    VNS vns(&parser);
    Solution vnsSol = vns.optimize(cwSol);
    assert(vnsSol.isValid() && "ERROR: VNS produjo solucion invalida.");

    CbcSolver cbc(&parser);
    Solution cbcSol = cbc.solve(vnsSol, TIME_LIMIT_SECONDS);
    assert(cbcSol.isValid() && "ERROR: CBC produjo solucion invalida.");

    cout << "\n=== RESUMEN DEL PIPELINE ===" << endl;
    cout << "Costo Clarke-Wright : " << cwSol.getTotalCost() << endl;
    cout << "Costo VNS           : " << vnsSol.getTotalCost()
         << "  (mejora sobre CW: "
         << cwSol.getTotalCost() - vnsSol.getTotalCost() << ")" << endl;
    cout << "Costo CBC           : " << cbcSol.getTotalCost()
         << "  (mejora sobre VNS: "
         << vnsSol.getTotalCost() - cbcSol.getTotalCost() << ")" << endl;
    cout << "Mejora total        : "
         << cwSol.getTotalCost() - cbcSol.getTotalCost()
         << "  ("
         << 100.0 * (cwSol.getTotalCost() - cbcSol.getTotalCost()) / cwSol.getTotalCost()
         << "%)" << endl;

    assert(cbcSol.getTotalCost() <= vnsSol.getTotalCost() &&
           "ERROR: CBC empeoro VNS en el pipeline completo.");
    assert(vnsSol.getTotalCost() <= cwSol.getTotalCost() &&
           "ERROR: VNS empeoro Clarke-Wright en el pipeline completo.");

    printSolution(cbcSol, "Solucion Final CBC");
    cout << "PASS: Pipeline CW -> VNS -> CBC valido y monotono." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 4: CBC con K-1 vehículos
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    string filename = (argc > 1) ? argv[1] : "sets/A-n32-k5.vrp";

    cout << "========================================" << endl;
    cout << "  TEST CBC - Branch & Cut COIN-OR" << endl;
    cout << "  Instancia  : " << filename << endl;
    cout << "  Warm start : Clarke-Wright + VNS" << endl;
    cout << "  Limite     : " << TIME_LIMIT_SECONDS << "s por llamada a CBC" << endl;
    cout << "========================================" << endl;

    Parser parser(filename);
    cout << "\nInstancia cargada:"
         << " N=" << parser.getDimension()
         << " Q=" << parser.getCapacity() << endl;

    // Cada test instancia su propio CbcSolver para aislar los resultados.
    // testCbcValidity(parser);
    // testCbcNotWorseVNS(parser);
    testFullPipeline(parser);

    cout << "\n========================================" << endl;
    cout << "  TODOS LOS TESTS PASARON" << endl;
    cout << "========================================" << endl;

    return 0;
}