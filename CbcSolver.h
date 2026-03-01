#ifndef CBC_SOLVER_H
#define CBC_SOLVER_H

#include "Parser.h"
#include "Solution.h"
#include "GreedyBuilder.h"
#include "VNS.h"               // Reemplaza KOpt como optimizador principal
#include "SubtourCut.h" // Cortes DFJ lazy
#include <vector>

class CbcSolver {
private:
    const Parser* parserData;
    int numClients;
    int numVariables;

    int getVarIndex(int i, int j) const;
    int getUIndex(int i) const;
    Solution convertToSolution(const double* solution) const;

public:
    explicit CbcSolver(const Parser* parser);

    // Resuelve el problema usando el motor exacto CBC de COIN-OR
    // con cortes DFJ, heurísticas internas y refinamiento VNS
    Solution solve(int numVehicles, double timeLimitSeconds = 120.0);
};

#endif // CBC_SOLVER_H