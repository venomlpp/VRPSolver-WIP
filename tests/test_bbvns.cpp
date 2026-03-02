// test_bbvns.cpp
// Variante experimental: B&B + CLP con warm start CW -> VNS en lugar de CW -> KOpt.
// Objetivo: verificar si un UB inicial más ajustado permite al B&B podar más
// agresivamente y encontrar mejores soluciones dentro del mismo límite de tiempo.
// Comparar resultados con test_bb para cuantificar el impacto del warm start.

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
// Warm start con VNS (30 iteraciones para no consumir demasiado
// tiempo antes de que empiece el B&B)
// ─────────────────────────────────────────────────────────────
Solution buildWarmStart(const Parser& parser) {
    GreedyBuilder builder(&parser);
    VNS vns(&parser);
    return vns.optimize(builder.buildSolution());
}

// ─────────────────────────────────────────────────────────────
// Test 1: BestFirst con warm start VNS produce solución válida
// ─────────────────────────────────────────────────────────────
void testBestFirstVNS(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 1: BestFirst + VNS produce solucion valida" << endl;
    cout << "========================================" << endl;

    Solution warmStart = buildWarmStart(parser);
    cout << "Warm start CW + VNS: " << warmStart.getTotalCost() << endl;

    BranchAndBound bb(&parser, warmStart);
    Solution result = bb.solveBestFirst(TIME_LIMIT_SECONDS);

    assert(result.isValid() &&
           "ERROR: BestFirst+VNS produjo solucion invalida.");
    assert(result.getTotalCost() <= warmStart.getTotalCost() &&
           "ERROR: BestFirst+VNS empeoro el warm start VNS.");

    printSolution(result, "BestFirst + VNS");
    cout << "PASS: BestFirst+VNS produjo solucion valida." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 2: DepthFirst con warm start VNS produce solución válida
// ─────────────────────────────────────────────────────────────
void testDepthFirstVNS(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 2: DepthFirst + VNS produce solucion valida" << endl;
    cout << "========================================" << endl;

    Solution warmStart = buildWarmStart(parser);
    cout << "Warm start CW + VNS: " << warmStart.getTotalCost() << endl;

    BranchAndBound bb(&parser, warmStart);
    Solution result = bb.solveDepthFirst(TIME_LIMIT_SECONDS);

    assert(result.isValid() &&
           "ERROR: DepthFirst+VNS produjo solucion invalida.");
    assert(result.getTotalCost() <= warmStart.getTotalCost() &&
           "ERROR: DepthFirst+VNS empeoro el warm start VNS.");

    printSolution(result, "DepthFirst + VNS");
    cout << "PASS: DepthFirst+VNS produjo solucion valida." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 3: Pipeline completo CW -> VNS -> BestFirst
// ─────────────────────────────────────────────────────────────
void testFullPipelineVNS(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 3: Pipeline CW -> VNS -> BestFirst" << endl;
    cout << "========================================" << endl;

    GreedyBuilder builder(&parser);
    Solution cwSol = builder.buildSolution();
    assert(cwSol.isValid() && "ERROR: Clarke-Wright invalido.");

    VNS vns(&parser);
    Solution vnsSol = vns.optimize(cwSol);
    assert(vnsSol.isValid() && "ERROR: VNS invalido.");

    BranchAndBound bb(&parser, vnsSol);
    auto t0 = chrono::steady_clock::now();
    Solution bbSol = bb.solveBestFirst(TIME_LIMIT_SECONDS);
    double elapsed = chrono::duration<double>(chrono::steady_clock::now() - t0).count();

    assert(bbSol.isValid() && "ERROR: BestFirst invalido en pipeline.");
    assert(bbSol.getTotalCost() <= vnsSol.getTotalCost() &&
           "ERROR: BestFirst empeoro VNS en pipeline.");

    cout << "\n=== RESUMEN PIPELINE (BestFirst + VNS) ===" << endl;
    cout << "Costo Clarke-Wright : " << cwSol.getTotalCost() << endl;
    cout << "Costo VNS           : " << vnsSol.getTotalCost()
         << "  (mejora: " << cwSol.getTotalCost() - vnsSol.getTotalCost() << ")" << endl;
    cout << "Costo BestFirst     : " << bbSol.getTotalCost()
         << "  (mejora: " << vnsSol.getTotalCost() - bbSol.getTotalCost() << ")" << endl;
    cout << "Mejora total        : " << cwSol.getTotalCost() - bbSol.getTotalCost()
         << "  (" << 100.0 * (cwSol.getTotalCost() - bbSol.getTotalCost()) / cwSol.getTotalCost()
         << "%)" << endl;
    cout << "Tiempo B&B          : " << elapsed << "s" << endl;

    printSolution(bbSol, "Solucion Final BestFirst+VNS");
    cout << "PASS: Pipeline CW -> VNS -> BestFirst valido y monotono." << endl;
}

// ─────────────────────────────────────────────────────────────
// Test 4: KOpt vs VNS como warm start — impacto en B&B
// El test más relevante: demuestra cuánto ayuda un UB más
// ajustado al B&B. Ambas estrategias usan BestFirst con el
// mismo límite de tiempo para aislar el efecto del warm start.
// ─────────────────────────────────────────────────────────────
void testWarmStartComparison(const Parser& parser) {
    cout << "\n========================================" << endl;
    cout << "TEST 4: KOpt vs VNS como warm start para B&B" << endl;
    cout << "  (BestFirst, mismo limite de tiempo)" << endl;
    cout << "========================================" << endl;

    // Warm start con KOpt
    GreedyBuilder builder(&parser);
    KOpt kopt(&parser);
    Solution koptWarm = kopt.optimize(builder.buildSolution());
    cout << "Warm start KOpt : " << koptWarm.getTotalCost() << endl;

    BranchAndBound bbKopt(&parser, koptWarm);
    auto t0 = chrono::steady_clock::now();
    Solution koptResult = bbKopt.solveBestFirst(TIME_LIMIT_SECONDS);
    double koptTime = chrono::duration<double>(chrono::steady_clock::now() - t0).count();

    assert(koptResult.isValid() && "ERROR: B&B+KOpt invalido.");

    // Warm start con VNS
    VNS vns(&parser);
    Solution vnsWarm = vns.optimize(builder.buildSolution());
    cout << "Warm start VNS  : " << vnsWarm.getTotalCost() << endl;

    BranchAndBound bbVns(&parser, vnsWarm);
    auto t1 = chrono::steady_clock::now();
    Solution vnsResult = bbVns.solveBestFirst(TIME_LIMIT_SECONDS);
    double vnsTime = chrono::duration<double>(chrono::steady_clock::now() - t1).count();

    assert(vnsResult.isValid() && "ERROR: B&B+VNS invalido.");

    cout << "\n=== IMPACTO DEL WARM START EN B&B ===" << endl;
    cout << "                  | Warm start | Resultado B&B | Mejora | Tiempo" << endl;
    cout << "  KOpt            | " << koptWarm.getTotalCost()
         << "       | " << koptResult.getTotalCost()
         << "           | " << koptWarm.getTotalCost() - koptResult.getTotalCost()
         << "      | " << koptTime << "s" << endl;
    cout << "  VNS (30 iter)   | " << vnsWarm.getTotalCost()
         << "       | " << vnsResult.getTotalCost()
         << "           | " << vnsWarm.getTotalCost() - vnsResult.getTotalCost()
         << "      | " << vnsTime << "s" << endl;

    if (vnsResult.getTotalCost() < koptResult.getTotalCost())
        cout << "\nCONCLUSION: VNS como warm start mejora B&B en "
             << koptResult.getTotalCost() - vnsResult.getTotalCost() << " unidades." << endl;
    else if (koptResult.getTotalCost() < vnsResult.getTotalCost())
        cout << "\nCONCLUSION: KOpt como warm start produce igual o mejor resultado "
             << "(VNS no ayuda en este caso)." << endl;
    else
        cout << "\nCONCLUSION: Ambos warm starts producen el mismo resultado final." << endl;

    cout << "PASS: Comparacion completada, ambas soluciones son validas." << endl;
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    string filename = (argc > 1) ? argv[1] : "sets/A-n32-k5.vrp";

    cout << "========================================" << endl;
    cout << "  TEST B&B+VNS - Branch & Bound + CLP + VNS" << endl;
    cout << "  Instancia  : " << filename << endl;
    cout << "  Warm start : Clarke-Wright + VNS (30 iter)" << endl;
    cout << "  Limite     : " << TIME_LIMIT_SECONDS << "s por estrategia" << endl;
    cout << "  (Tiempo total estimado: hasta ~"
         << (int)(TIME_LIMIT_SECONDS * 3 / 60 + 2) << " minutos)" << endl;
    cout << "========================================" << endl;

    Parser parser(filename);
    cout << "\nInstancia cargada:"
         << " N=" << parser.getDimension()
         << " Q=" << parser.getCapacity() << endl;

    // testBestFirstVNS(parser);
    // testDepthFirstVNS(parser);
    testFullPipelineVNS(parser);
    // testWarmStartComparison(parser);

    cout << "\n========================================" << endl;
    cout << "  TODOS LOS TESTS PASARON" << endl;
    cout << "========================================" << endl;

    return 0;
}