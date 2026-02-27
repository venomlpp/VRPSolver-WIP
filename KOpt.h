#ifndef KOPT_H
#define KOPT_H

#include "Solution.h"
#include "Parser.h"
#include "Route.h"

class KOpt {
private:
    const Parser* parserData;

    // Método privado central: Toma una ruta y la optimiza exhaustivamente usando 3-OPT.
    // Retorna la misma ruta optimizada.
    Route optimizeRoute(const Route& route);

public:
    // Constructor que recibe el entorno (matriz de distancias, etc.)
    explicit KOpt(const Parser* parser);

    // Método principal: Toma la solución del Greedy y le aplica 3-OPT a TODAS sus rutas
    Solution optimize(const Solution& initialSolution);

    ~KOpt();
};

#endif // KOPT_H