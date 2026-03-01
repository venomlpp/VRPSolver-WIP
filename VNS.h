#ifndef VNS_H
#define VNS_H

#include "Parser.h"
#include "Solution.h"
#include "Route.h"
#include "KOpt.h"
#include <random>

// ============================================================
// VNS - Variable Neighborhood Search para CVRP
//
// Aplica operadores inter-ruta en orden creciente de perturbación.
// Si un operador mejora la solución, reinicia desde el primero.
// Si ninguno mejora, retorna el mejor encontrado.
//
// Vecindades en orden:
//   N1 - Relocate:    mover 1 cliente a otra ruta
//   N2 - Swap:        intercambiar 1 cliente entre dos rutas
//   N3 - Or-Opt-2:    mover 2 clientes consecutivos a otra ruta
//   N4 - Or-Opt-3:    mover 3 clientes consecutivos a otra ruta
// Después de cada mejora inter-ruta se aplica 3-OPT intra-ruta.
// ============================================================
class VNS {
public:
    explicit VNS(const Parser* parser);

    // Método principal: aplica VNS a la solución recibida
    Solution optimize(const Solution& initialSolution);

private:
    const Parser* parserData;

    // Vecindad 1: mover un cliente de routeFrom a routeTo en la mejor posición
    bool neighborhoodRelocate(Solution& sol);

    // Vecindad 2: intercambiar un cliente de routeA con uno de routeB
    bool neighborhoodSwap(Solution& sol);

    // Vecindad 3 y 4: mover segmento de `segLen` clientes a otra ruta
    bool neighborhoodOrOpt(Solution& sol, int segLen);

    // NUEVO: fase de shaking
    Solution shake(const Solution& sol, int intensity, std::mt19937& rng) const;

    // Calcula el costo de insertar `clientId` entre `prev` y `next`
    int insertionCost(int prev, int clientId, int next) const;

    // Calcula el ahorro de eliminar `clientId` de entre `prev` y `next`
    int removalSaving(int prev, int clientId, int next) const;
};

#endif // VNS_H