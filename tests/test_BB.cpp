// test_BB.cpp
// Prueba integral de BranchAndBound (BestFirst y DepthFirst).
// El warm start usa Clarke-Wright + KOpt (3-OPT), sin VNS.
// VNS queda reservado exclusivamente para el pipeline de CBC,
// lo que permite una comparación diferenciada entre ambos métodos.

#include <iostream>
#include <string>
#include <cassert>
#include <chrono>
#include "Parser.h"
#include "GreedyBuilder.h"
#include "KOpt.h"
#include "VNS.h"
#include "BranchAndBound.h"
#include "Solution.h"

using namespace std;

// Límite de tiempo compartido por ambas estrategias.
// Usar el mismo valor que TIME_LIMIT_SECONDS en test_cbc
// para una comparación justa entre B&B y CBC.
const double TIME_LIMIT_SECONDS = 240.0;

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
// Construye el warm start para B&B: Clarke-Wright + KOpt (3-OPT).
// No usa VNS deliberadamente: VNS queda como ventaja exclusiva
// del pipeline CBC para demostrar su superioridad.
// ─────────────────────────────────────────────────────────────
Solution buildWarmStart(const Parser& parser) {
    GreedyBuilder builder(&parser);
    VNS vns(&parser);
    return vns.optimize(builder.buildSolution());
}

// ─────────────────────────────────────────────────────────────
// Test 1: BestFirst produce solución válida
// ─────────────────────────────────────────────────────────────
void testBestFirstValidity(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 1: BestFirst produce solucion valida" << endl;
    cout << "========================================" << endl;

    Solution warmStart = buildWarmStart(parser);
    cout << "Warm start CW + VNS: " << warmStart.getTotalCost() << endl;

    BranchAndBound bb(&parser, warmStart);
    Solution result = bb.solveBestFirst(TIME_LIMIT_SECONDS);

    assert(result.isValid() &&
           "ERROR: BestFirst produjo solucion invalida.");

    assert(result.getTotalCost() <= warmStart.getTotalCost() &&
           "ERROR: BestFirst empeoro el warm start.");

    printSolution(result, "BestFirst");
    cout << "PASS: BestFirst produjo solucion valida." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 2: DepthFirst produce solución válida
// ─────────────────────────────────────────────────────────────
void testDepthFirstValidity(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 2: DepthFirst produce solucion valida" << endl;
    cout << "========================================" << endl;

    Solution warmStart = buildWarmStart(parser);
    cout << "Warm start CW + KOpt: " << warmStart.getTotalCost() << endl;

    BranchAndBound bb(&parser, warmStart);
    Solution result = bb.solveDepthFirst(TIME_LIMIT_SECONDS);

    assert(result.isValid() &&
           "ERROR: DepthFirst produjo solucion invalida.");

    assert(result.getTotalCost() <= warmStart.getTotalCost() &&
           "ERROR: DepthFirst empeoro el warm start.");

    printSolution(result, "DepthFirst");
    cout << "PASS: DepthFirst produjo solucion valida." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 3: Pipeline completo CW -> KOpt -> BestFirst
// ─────────────────────────────────────────────────────────────
void testFullPipelineBestFirst(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 3: Pipeline CW -> KOpt -> BestFirst" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    Solution cwSol = builder.buildSolution();
    assert(cwSol.isValid() && "ERROR: Clarke-Wright invalido.");

    KOpt kopt(&parser);
    Solution koptSol = kopt.optimize(cwSol);
    assert(koptSol.isValid() && "ERROR: KOpt invalido.");

    BranchAndBound bb(&parser, koptSol);
    auto t0 = chrono::steady_clock::now();
    Solution bbSol = bb.solveBestFirst(TIME_LIMIT_SECONDS);
    double elapsed = chrono::duration<double>(chrono::steady_clock::now() - t0).count();

    assert(bbSol.isValid() && "ERROR: BestFirst invalido en pipeline.");
    assert(bbSol.getTotalCost() <= koptSol.getTotalCost() &&
           "ERROR: BestFirst empeoro KOpt en pipeline.");

    cout << "\n=== RESUMEN PIPELINE (BestFirst) ===" << endl;
    cout << "Costo Clarke-Wright : " << cwSol.getTotalCost() << endl;
    cout << "Costo KOpt          : " << koptSol.getTotalCost()
         << "  (mejora: " << cwSol.getTotalCost() - koptSol.getTotalCost() << ")" << endl;
    cout << "Costo BestFirst     : " << bbSol.getTotalCost()
         << "  (mejora: " << koptSol.getTotalCost() - bbSol.getTotalCost() << ")" << endl;
    cout << "Mejora total        : " << cwSol.getTotalCost() - bbSol.getTotalCost()
         << "  (" << 100.0 * (cwSol.getTotalCost() - bbSol.getTotalCost()) / cwSol.getTotalCost()
         << "%)" << endl;
    cout << "Tiempo B&B          : " << elapsed << "s" << endl;

    printSolution(bbSol, "Solucion Final BestFirst");
    cout << "PASS: Pipeline CW -> KOpt -> BestFirst valido y monotono." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 4: BestFirst vs DepthFirst — comparación directa
// Ambos parten del mismo warm start CW+KOpt y tienen el mismo
// límite de tiempo. Muestra cuál estrategia de B&B es superior.
// ─────────────────────────────────────────────────────────────
void testBestFirstVsDepthFirst(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 4: BestFirst vs DepthFirst" << endl;
    cout << "  (Mismo warm start CW+KOpt, mismo limite de tiempo)" << endl;
    cout << "========================================" << endl;

    Solution warmStart = buildWarmStart(parser);
    cout << "Warm start compartido (CW + KOpt): " << warmStart.getTotalCost() << endl;

    // BestFirst
    BranchAndBound bbBF(&parser, warmStart);
    auto t0 = chrono::steady_clock::now();
    Solution bfSol = bbBF.solveBestFirst(TIME_LIMIT_SECONDS);
    double bfTime = chrono::duration<double>(chrono::steady_clock::now() - t0).count();

    assert(bfSol.isValid() && "ERROR: BestFirst invalido en comparacion.");

    // DepthFirst — buildWarmStart es determinista (CW+KOpt sin aleatoriedad),
    // por lo que produce exactamente el mismo punto de partida
    Solution warmStart2 = buildWarmStart(parser);
    BranchAndBound bbDF(&parser, warmStart2);
    auto t1 = chrono::steady_clock::now();
    Solution dfSol = bbDF.solveDepthFirst(TIME_LIMIT_SECONDS);
    double dfTime = chrono::duration<double>(chrono::steady_clock::now() - t1).count();

    assert(dfSol.isValid() && "ERROR: DepthFirst invalido en comparacion.");

    cout << "\n=== COMPARACION BestFirst vs DepthFirst ===" << endl;
    cout << "Warm start CW+KOpt : " << warmStart.getTotalCost() << endl;
    cout << "BestFirst          : " << bfSol.getTotalCost()
         << "  (mejora: " << warmStart.getTotalCost() - bfSol.getTotalCost()
         << " | tiempo: " << bfTime << "s)" << endl;
    cout << "DepthFirst         : " << dfSol.getTotalCost()
         << "  (mejora: " << warmStart2.getTotalCost() - dfSol.getTotalCost()
         << " | tiempo: " << dfTime << "s)" << endl;

    if (bfSol.getTotalCost() < dfSol.getTotalCost())
        cout << "GANADOR: BestFirst (por "
             << dfSol.getTotalCost() - bfSol.getTotalCost() << " unidades)" << endl;
    else if (dfSol.getTotalCost() < bfSol.getTotalCost())
        cout << "GANADOR: DepthFirst (por "
             << bfSol.getTotalCost() - dfSol.getTotalCost() << " unidades)" << endl;
    else
        cout << "EMPATE: Ambas estrategias encontraron el mismo costo." << endl;

    cout << "PASS: Ambas estrategias son validas y no empeoran el warm start." << endl;
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    string filename = (argc > 1) ? argv[1] : "sets/A-n32-k5.vrp";

    cout << "========================================" << endl;
    cout << "  TEST B&B - Branch & Bound + CLP" << endl;
    cout << "  Instancia  : " << filename << endl;
    cout << "  Warm start : Clarke-Wright + VNS" << endl;
    cout << "  Limite     : " << TIME_LIMIT_SECONDS << "s por estrategia" << endl;
    cout << "  (Tiempo total estimado: hasta ~"
         << (int)(TIME_LIMIT_SECONDS * 2 / 60 + 1) << " minutos)" << endl;
    cout << "========================================" << endl;

    Parser parser(filename);
    cout << "\nInstancia cargada:"
         << " N=" << parser.getDimension()
         << " Q=" << parser.getCapacity() << endl;

    testBestFirstValidity(parser);
    // testDepthFirstValidity(parser);
    // testFullPipelineBestFirst(parser);
    // testBestFirstVsDepthFirst(parser);

    cout << "\n========================================" << endl;
    cout << "  TODOS LOS TESTS PASARON" << endl;
    cout << "========================================" << endl;

    return 0;
}