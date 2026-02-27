#ifndef GREEDY_BUILDER_H
#define GREEDY_BUILDER_H

#include "Parser.h"
#include "Solution.h"
#include <vector>

class GreedyBuilder {
private:
    const Parser* parserData;

public:
    // Constructor: Solo requiere el parser para acceder a los datos
    explicit GreedyBuilder(const Parser* parser);

    // Método principal: Construye y retorna una solución válida
    Solution buildSolution();

    // Destructor
    ~GreedyBuilder();
};

#endif // GREEDY_BUILDER_H