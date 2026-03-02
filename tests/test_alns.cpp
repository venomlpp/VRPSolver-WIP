// test_alns.cpp
// Pipeline: CW → VNS → ALNS+LNS-MIP → VNS final

#include <iostream>
#include <string>
#include <chrono>
#include "Parser.h"
#include "GreedyBuilder.h"
#include "VNS.h"
#include "ALNS.h"
#include "CbcSolver.h"

using namespace std;
using chrono::steady_clock;
using chrono::duration;

int main(int argc, char* argv[]) {

    string filename = (argc > 1) ? argv[1] : "sets/A-n32-k5.vrp";
    double totalTime    = (argc > 2) ? stod(argv[2]) : 60.0;

    cout << "========================================\n";
    cout << "  TEST LNS-MIP (ALNS + CBC Subproblema)\n";
    cout << "  Instancia  : " << filename          << "\n";
    cout << "  Tiempo total: " << totalTime << "s\n";
    cout << "========================================\n";

    // ── 1. Cargar instancia ───────────────────────────────────────────────
    Parser parser(filename);
    cout << "Instancia cargada: N=" << parser.getDimension()
         << " Q=" << parser.getCapacity() << "\n\n";

    auto t0 = steady_clock::now();

    // ── 2. Clarke-Wright ──────────────────────────────────────────────────
    GreedyBuilder builder(&parser);
    Solution cwSol = builder.buildSolution();
    cout << "Clarke-Wright    : " << cwSol.getTotalCost() << "\n";

    // ── 3. VNS inicial (warm start) ───────────────────────────────────────
    VNS vns(&parser);
    Solution vnsSol = vns.optimize(cwSol, 100);
    cout << "VNS warm start   : " << vnsSol.getTotalCost()
         << "  (mejora: " << cwSol.getTotalCost() - vnsSol.getTotalCost() << ")\n";

    // ── 4. ALNS + LNS-MIP ────────────────────────────────────────────────

    cout << "\n[ALNS] Tiempo disponible: " << totalTime << "s\n";

    CbcSolver cbc(&parser);
    ALNS alns(&parser, &cbc);

    // Calibración para instancias medianas (ajustar según tamaño)
    int n = parser.getDimension() - 1; // clientes sin depot
    int destroySize = max(5, min(20, n / 6)); // ~16% de los clientes
    alns.setDestroySize(destroySize);
    alns.setVnsIterations(10);
    cout << "[ALNS] destroySize=" << destroySize << "\n\n";

    Solution alnsSol = alns.optimize(vnsSol, totalTime);

    // ── 5. VNS final ─────────────────────────────────────────────────────
    Solution finalSol = vns.optimize(alnsSol, 50);

    double totalElapsed = duration<double>(steady_clock::now() - t0).count();

    // ── 6. Reporte ────────────────────────────────────────────────────────
    cout << "\n========================================\n";
    cout << "  RESUMEN\n";
    cout << "========================================\n";
    cout << "Clarke-Wright    : " << cwSol.getTotalCost()   << "\n";
    cout << "VNS warm start   : " << vnsSol.getTotalCost()  << "\n";
    cout << "ALNS+LNS-MIP     : " << alnsSol.getTotalCost() << "\n";
    cout << "VNS final        : " << finalSol.getTotalCost() << "\n";
    cout << "Mejora total     : "
         << cwSol.getTotalCost() - finalSol.getTotalCost()
         << "  ("
         << 100.0 * (cwSol.getTotalCost() - finalSol.getTotalCost())
                  / cwSol.getTotalCost()
         << "%)\n";
    cout << "Tiempo total     : " << totalElapsed << "s\n";

    // Validación
    if (!finalSol.isValid()) {
        cout << "\nFAIL: Solucion final invalida.\n";
        return 1;
    }

    cout << "\n--- Solucion Final ---\n";
    cout << "Costo total : " << finalSol.getTotalCost() << "\n";
    cout << "Vehiculos   : " << finalSol.getRoutes().size() << "\n";
    for (size_t i = 0; i < finalSol.getRoutes().size(); ++i) {
        const auto& route = finalSol.getRoutes()[i];
        const auto& path  = route.getPath();
        cout << "  Vehiculo " << i+1
             << " | Carga: " << route.getCurrentLoad()
             << " | Costo: " << route.getTotalCost()
             << " | Ruta: [ ";
        for (int id : path) cout << id << " ";
        cout << "]\n";
    }

    cout << "\nPASS: Solucion valida.\n";
    return 0;
}