#ifndef KOPT_H
#define KOPT_H

#include "Solution.h"
#include "Parser.h"
#include "Route.h"

/*
 * Clase KOpt
 * Descripción: Implementa la heurística de búsqueda local intra-ruta 3-OPT.
 * Se encarga de refinar iterativamente las rutas rompiendo hasta tres aristas 
 * y reconectando los segmentos resultantes para encontrar un mínimo local.
 */
class KOpt {
private:
    const Parser* parserData;

    Route optimizeRoute(const Route& route);

public:
    explicit KOpt(const Parser* parser);

    Solution optimize(const Solution& initialSolution);

    ~KOpt();
};

#endif // KOPT_H