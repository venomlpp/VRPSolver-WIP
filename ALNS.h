#ifndef ALNS_H
#define ALNS_H

#include <vector>
#include <random>
#include "Parser.h"
#include "Solution.h"
#include "Route.h"
#include "VNS.h"

// Forward declaration para evitar include circular
class CbcSolver;

// ─────────────────────────────────────────────────────────────
// ALNS — Adaptive Large Neighborhood Search con CBC como repair
//
// Pipeline:
//   optimize(initialSol, timeLimit):
//     Loop hasta agotar tiempo:
//       1. destroy: extraer k clientes de la solución actual
//       2. cbcRepair: CBC resuelve reinserción óptima del subproblema
//       3. vnsPolish: VNS corto pule el resultado
//       4. aceptar si mejora (hill climbing)
//     return mejor solución encontrada
// ─────────────────────────────────────────────────────────────
class ALNS {
public:
    ALNS(const Parser* parser, CbcSolver* cbc);

    // Punto de entrada principal
    // initialSol : solución de partida (e.g. resultado de VNS)
    // timeLimitSeconds : tiempo total disponible para ALNS
    Solution optimize(const Solution& initialSol, double timeLimitSeconds);

    // Tamaño del destroy (número de clientes a extraer por iteración)
    // Por defecto 15; se puede ajustar antes de llamar a optimize()
    void setDestroySize(int k) { destroySize = k; }

    // Iteraciones de VNS post-proceso por ciclo ALNS
    void setVnsIterations(int iter) { vnsIter = iter; }

private:
    const Parser* parserData;
    CbcSolver*    cbcSolver;
    VNS           vns;

    int destroySize = 15;   // clientes extraídos por iteración
    int vnsIter     = 10;   // iteraciones VNS de pulido post-repair

    std::mt19937 rng;       // generador aleatorio con seed fija

    // ── Operadores Destroy ────────────────────────────────────

    // Shaw removal: extrae clientes geográficamente relacionados.
    // Parte de un cliente aleatorio y agrega los más cercanos a él.
    // Favorece la exploración de intercambios de clientes vecinos.
    std::vector<int> shawRemoval(const Solution& sol, int k);

    // Worst removal: extrae los clientes con mayor "savings" negativo,
    // es decir, los que más encarecen sus rutas actuales.
    // Favorece la corrección de inserciones pobres.
    std::vector<int> worstRemoval(const Solution& sol, int k);

    // Random removal: extrae k clientes al azar.
    // Garantiza diversificación cuando los otros operadores se estancan.
    std::vector<int> randomRemoval(const Solution& sol, int k);

    // ── Helpers ───────────────────────────────────────────────

    // Calcula el costo de insertar el cliente clientId entre prev y next
    double insertionCost(int clientId, int prev, int next) const;

    // Calcula el ahorro de REMOVER clientId de su posición actual
    // (costo de la ruta sin él - costo de la ruta con él, negado)
    double removalSavings(int clientId,
                          int prev, int next) const;
};

#endif // ALNS_H