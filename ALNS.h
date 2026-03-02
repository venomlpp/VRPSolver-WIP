#ifndef ALNS_H
#define ALNS_H

#include <vector>
#include <random>
#include "Parser.h"
#include "Solution.h"
#include "Route.h"
#include "VNS.h"

class CbcSolver;

/*
 * Clase ALNS
 * Descripción: Implementa la metaheurística Adaptive Large Neighborhood Search.
 * Coordina la destrucción de componentes de la solución y delega la reparación 
 * óptima de dichos subproblemas al solver exacto CBC.
 */
class ALNS {
public:
    ALNS(const Parser* parser, CbcSolver* cbc);

    Solution optimize(const Solution& initialSol, double timeLimitSeconds);

    void setDestroySize(int k) { destroySize = k; }
    void setVnsIterations(int iter) { vnsIter = iter; }

private:
    const Parser* parserData;
    CbcSolver* cbcSolver;
    VNS           vns;

    int destroySize = 15;
    int vnsIter     = 10;

    std::mt19937 rng;

    // ── Operadores Destroy ────────────────────────────────────
    std::vector<int> shawRemoval(const Solution& sol, int k);
    std::vector<int> worstRemoval(const Solution& sol, int k);
    std::vector<int> randomRemoval(const Solution& sol, int k);

    // ── Helpers ───────────────────────────────────────────────
    double insertionCost(int clientId, int prev, int next) const;
    double removalSavings(int clientId, int prev, int next) const;
};

#endif // ALNS_H