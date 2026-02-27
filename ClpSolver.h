#ifndef CLP_SOLVER_H
#define CLP_SOLVER_H

#include "Parser.h"
#include <coin/ClpSimplex.hpp>

class ClpSolver {
private:
    const Parser* parserData;
    int numClients;
    int numVariables;

    int getVarIndex(int i, int j) const;

public:
    explicit ClpSolver(const Parser* parser);
    
    // Resuelve el problema relajado y muestra la "sopa de decimales"
    void solveRelaxation(int numVehicles);
};

#endif // CLP_SOLVER_H