#ifndef GREEDY_BUILDER_H
#define GREEDY_BUILDER_H

#include "Parser.h"
#include "Solution.h"
#include <vector>

/*
 * Clase GreedyBuilder
 * Descripción: Implementa el algoritmo constructivo heurístico de Ahorros 
 * de Clarke-Wright para generar una solución inicial factible y de buena calidad
 * para el problema CVRP.
 */
class GreedyBuilder {
private:
    const Parser* parserData;

public:
    explicit GreedyBuilder(const Parser* parser);
    Solution buildSolution();
    ~GreedyBuilder();
};

#endif // GREEDY_BUILDER_H