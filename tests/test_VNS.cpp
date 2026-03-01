// test_VNS.cpp
// Prueba integral de la clase VNS.
// Verifica que VNS mejore la solución inicial de Clarke-Wright
// y que la solución resultante sea válida (sin subtours, sin violaciones
// de capacidad, todos los clientes visitados exactamente una vez).

#include <iostream>
#include <string>
#include <cassert>
#include "Parser.h"
#include "GreedyBuilder.h"
#include "KOpt.h"
#include "VNS.h"
#include "Solution.h"

using namespace std;

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
// Test 1: VNS no empeora una solución ya óptima localmente
// Se aplica KOpt primero para llegar a un mínimo local 3-OPT,
// luego VNS debería mantener o mejorar ese resultado.
// ─────────────────────────────────────────────────────────────
void testVNSNotWorse(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 1: VNS no empeora solucion KOpt" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    Solution cw = builder.buildSolution();

    KOpt kopt(&parser);
    Solution koptSol = kopt.optimize(cw);

    VNS vns(&parser);
    Solution vnsSol = vns.optimize(koptSol);

    cout << "Costo KOpt (entrada a VNS) : " << koptSol.getTotalCost() << endl;
    cout << "Costo VNS  (salida)        : " << vnsSol.getTotalCost() << endl;

    assert(vnsSol.getTotalCost() <= koptSol.getTotalCost() &&
           "ERROR: VNS empeoro la solucion KOpt.");

    assert(vnsSol.isValid() &&
           "ERROR: VNS produjo una solucion invalida.");

    cout << "PASS: VNS no empeoro KOpt y la solucion es valida." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 2: VNS mejora Clarke-Wright directamente
// Mide la mejora total del pipeline CW -> VNS.
// ─────────────────────────────────────────────────────────────
void testVNSImprovesGreedy(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 2: VNS mejora Clarke-Wright" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    Solution cw = builder.buildSolution();

    VNS vns(&parser);
    Solution vnsSol = vns.optimize(cw);

    printSolution(cw,     "Clarke-Wright (sin optimizar)");
    printSolution(vnsSol, "VNS (sobre Clarke-Wright)");

    double improvement = cw.getTotalCost() - vnsSol.getTotalCost();
    double pct = 100.0 * improvement / cw.getTotalCost();

    cout << "\nMejora absoluta : " << improvement << endl;
    cout << "Mejora relativa : " << pct << "%" << endl;

    assert(vnsSol.getTotalCost() <= cw.getTotalCost() &&
           "ERROR: VNS empeoro Clarke-Wright.");

    assert(vnsSol.isValid() &&
           "ERROR: VNS produjo una solucion invalida sobre CW.");

    cout << "PASS: VNS mejora o iguala Clarke-Wright." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 3: Pipeline completo CW -> KOpt -> VNS
// Muestra los tres escalones de mejora y verifica validez en cada paso.
// ─────────────────────────────────────────────────────────────
void testFullPipeline(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 3: Pipeline CW -> KOpt -> VNS" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    Solution cw = builder.buildSolution();
    assert(cw.isValid() && "ERROR: Clarke-Wright produjo solucion invalida.");

    KOpt kopt(&parser);
    Solution koptSol = kopt.optimize(cw);
    assert(koptSol.isValid() && "ERROR: KOpt produjo solucion invalida.");

    VNS vns(&parser);
    Solution vnsSol = vns.optimize(koptSol);
    assert(vnsSol.isValid() && "ERROR: VNS produjo solucion invalida.");

    cout << "Costo Clarke-Wright : " << cw.getTotalCost() << endl;
    cout << "Costo KOpt          : " << koptSol.getTotalCost()
         << "  (mejora: " << cw.getTotalCost() - koptSol.getTotalCost() << ")" << endl;
    cout << "Costo VNS           : " << vnsSol.getTotalCost()
         << "  (mejora: " << koptSol.getTotalCost() - vnsSol.getTotalCost() << ")" << endl;
    cout << "Mejora total        : " << cw.getTotalCost() - vnsSol.getTotalCost()
         << "  ("
         << 100.0 * (cw.getTotalCost() - vnsSol.getTotalCost()) / cw.getTotalCost()
         << "%)" << endl;

    // Verificar que VNS al menos no empeora KOpt
    assert(vnsSol.getTotalCost() <= koptSol.getTotalCost() &&
           "ERROR: VNS empeoro KOpt en el pipeline completo.");

    cout << "PASS: Pipeline CW -> KOpt -> VNS valido y monotono." << endl;

    // Mostrar solución final detallada
    printSolution(vnsSol, "Solucion Final VNS");
}

// ─────────────────────────────────────────────────────────────
// Test 4: Idempotencia
// Aplicar VNS dos veces sobre la misma solución no debe empeorarla.
// ─────────────────────────────────────────────────────────────
void testIdempotence(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 4: Idempotencia de VNS" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    Solution cw = builder.buildSolution();

    VNS vns(&parser);
    Solution first  = vns.optimize(cw);
    Solution second = vns.optimize(first);

    cout << "Costo primera pasada VNS  : " << first.getTotalCost() << endl;
    cout << "Costo segunda pasada VNS  : " << second.getTotalCost() << endl;

    assert(second.getTotalCost() <= first.getTotalCost() &&
           "ERROR: Segunda pasada de VNS empeoro la primera.");

    assert(second.isValid() &&
           "ERROR: Segunda pasada de VNS produjo solucion invalida.");

    cout << "PASS: VNS es idempotente (segunda pasada no empeora)." << endl;
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // Permite pasar el archivo VRP como argumento; si no, usa el de ejemplo
    string filename = (argc > 1) ? argv[1] : "sets/A-n32-k5.vrp";

    cout << "========================================" << endl;
    cout << "  TEST VNS - Variable Neighborhood Search" << endl;
    cout << "  Instancia: " << filename << endl;
    cout << "========================================" << endl;

    Parser parser(filename);
    cout << "\nInstancia cargada:"
         << " N=" << parser.getDimension()
         << " Q=" << parser.getCapacity() << endl;

    // Ejecutar todos los tests
    testVNSImprovesGreedy(parser);
    testVNSNotWorse(parser);
    testFullPipeline(parser);
    testIdempotence(parser);

    cout << "\n========================================" << endl;
    cout << "  TODOS LOS TESTS PASARON" << endl;
    cout << "========================================" << endl;

    return 0;
}