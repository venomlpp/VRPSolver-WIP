#ifndef CBC_SOLVER_H
#define CBC_SOLVER_H

#include "Parser.h"
#include "Solution.h"
#include "GreedyBuilder.h"
#include "KOpt.h"
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
    Solution solve(int numVehicles);
};

#endif // CBC_SOLVER_H