#ifndef VNS_H
#define VNS_H

#include "Parser.h"
#include "Solution.h"
#include "Route.h"
#include "KOpt.h"
#include <random>

/*
 * Clase VNS
 * Descripción: Implementa la metaheurística Variable Neighborhood Search (VNS) para el CVRP.
 * Aplica sistemáticamente estructuras de vecindad inter-ruta (Relocate, Swap, Or-Opt)
 * combinadas con optimización intra-ruta (3-OPT) y fases de agitación (Shaking) 
 * para escapar de óptimos locales.
 */
class VNS {
public:
    explicit VNS(const Parser* parser);

    Solution optimize(const Solution& initialSolution, int maxIter = 100);

private:
    const Parser* parserData;

    bool neighborhoodRelocate(Solution& sol);

    bool neighborhoodSwap(Solution& sol);

    bool neighborhoodOrOpt(Solution& sol, int segLen);

    Solution shake(const Solution& sol, int intensity, std::mt19937& rng) const;

    int insertionCost(int prev, int clientId, int next) const;

    int removalSaving(int prev, int clientId, int next) const;
};

#endif // VNS_H