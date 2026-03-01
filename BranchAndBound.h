#ifndef BRANCH_AND_BOUND_H
#define BRANCH_AND_BOUND_H

#include <vector>
#include <queue>
#include <stack>
#include "Parser.h"
#include "Solution.h"
#include "KOpt.h"
#include <coin/ClpSimplex.hpp> 

struct BBNode {
    int level;                           
    double lowerBound;                   
    std::vector<double> lowerBounds;     
    std::vector<double> upperBounds;     

    bool operator<(const BBNode& other) const {
        return lowerBound > other.lowerBound;
    }
};

class BranchAndBound {
private:
    const Parser* parserData;
    double globalUpperBound;             
    Solution bestSolution;               
    
    int numClients;                      
    int numVariables;                    

    void buildBaseModel(ClpSimplex& model);
    int getVarIndex(int i, int j) const;
    int getMostFractionalVariable(const double* solution) const;
    Solution convertToSolution(const double* solution) const;
    Solution roundingHeuristic(const double* solution) const;
    Solution lpGuidedConstruction(const double* lpSol) const;

    // NUEVO: Algoritmo de grafos para detectar trampas en la solución entera
    std::vector<std::vector<int>> findInvalidSets(const double* solution) const;

public:
    BranchAndBound(const Parser* parser, const Solution& initialSolution);
    Solution solveBestFirst(double timeLimitSeconds = 120.0);
    Solution solveDepthFirst(double timeLimitSeconds = 120.0);
    ~BranchAndBound();
};

#endif // BRANCH_AND_BOUND_H