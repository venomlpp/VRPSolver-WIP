#ifndef SUBTOUR_CUT_H
#define SUBTOUR_CUT_H

#include <vector>
#include <coin/CglCutGenerator.hpp>  // Clase base del plugin CBC
#include <coin/OsiSolverInterface.hpp>
#include <coin/OsiCuts.hpp>
#include <coin/OsiRowCut.hpp>
#include "Parser.h"

// =============================================================================
// SubtourCutGenerator
// =============================================================================
// Generador de cortes DFJ (Dantzig-Fulkerson-Johnson) registrado en CBC como
// plugin CglCutGenerator. CBC lo invoca automáticamente en cada nodo del
// árbol Branch & Bound.
//
// ¿Por qué es necesario?
//   El modelo MTZ en CbcSolver evita subtours con variables U_i, pero la
//   relajación LP resultante es muy débil (LB ~603 para A-n32-k5 cuando el
//   óptimo es ~784). Eso obliga a CBC a explorar un árbol enorme.
//
//   Los cortes DFJ son mucho más fuertes: para cada subconjunto S de nodos
//   que forma un subtour o viola la capacidad Q, agregan:
//       sum_{i in S, j in S, i!=j}  x_ij  <=  |S| - k_s
//   donde  k_s = ceil( demanda(S) / Q )  >= 1.
//
//   Aplicados como "lazy cuts" (solo cuando se detecta una violación) suben
//   el LB rápidamente sin inflar el tamaño del modelo en el nodo raíz.
//
// Uso en CbcSolver.cpp (insertar antes de model.branchAndBound()):
//
//   SubtourCutGenerator subtourGen(parserData, numClients);
//   model.addCutGenerator(&subtourGen, 1, "SubtourDFJ");
//   // El segundo argumento (1) significa: aplicar en CADA nodo.
//   // Usar -1 para aplicar solo en la raíz, o 10 para cada 10 niveles.
// =============================================================================

class SubtourCutGenerator : public CglCutGenerator {
public:
    // -------------------------------------------------------------------------
    // Constructor
    //   parser     : puntero al parser (demandas, capacidad Q)
    //   numClients : dimensión total incluyendo bodega (índice interno 0)
    // -------------------------------------------------------------------------
    SubtourCutGenerator(const Parser* parser, int numClients);

    // -------------------------------------------------------------------------
    // generateCuts  [override obligatorio de CglCutGenerator]
    //
    // CBC llama a este método en cada nodo. Lee la solución LP del nodo,
    // detecta conjuntos S inválidos y agrega los cortes DFJ al objeto `cs`.
    // CBC procesa después automáticamente los cortes insertados en `cs`.
    // -------------------------------------------------------------------------
    void generateCuts(const OsiSolverInterface& si,
                      OsiCuts& cs,
                      const CglTreeInfo info = CglTreeInfo()) override;

    // -------------------------------------------------------------------------
    // clone  [override obligatorio de CglCutGenerator]
    //
    // CBC clona el generador al distribuir trabajo. Retorna una copia en heap;
    // CBC toma ownership y la libera.
    // -------------------------------------------------------------------------
    CglCutGenerator* clone() const override;

    ~SubtourCutGenerator() override = default;

private:
    const Parser* parserData;  // Acceso a demandas y capacidad Q
    int n;                     // Número de nodos (bodega = índice 0)

    // Convierte el par (i,j) a índice lineal. DEBE coincidir con
    // CbcSolver::getVarIndex (= i * numClients + j).
    inline int getVarIndex(int i, int j) const { return i * n + j; }

    // -------------------------------------------------------------------------
    // findInvalidSets
    //
    // Dado el vector `sol` (solución LP del nodo actual, tamaño >= n*n):
    //
    //   Paso 1 – Grafo de soporte:
    //     Considera activa la arista (i→j) si sol[i*n+j] > threshold.
    //     threshold=0.5  →  detección exacta para soluciones enteras.
    //     threshold=1e-4 →  detección agresiva para soluciones fraccionales.
    //
    //   Paso 2 – BFS desde la bodega (nodo 0):
    //     Marca todos los nodos alcanzables desde 0.
    //
    //   Paso 3 – Subtours aislados:
    //     Nodos no alcanzados forman componentes sin conexión a la bodega.
    //     Cada componente es un conjunto inválido.
    //
    //   Paso 4 – Violaciones de capacidad:
    //     Para rutas que SÍ parten de la bodega, suma las demandas de los
    //     nodos en la ruta. Si la suma > Q, esa ruta es un conjunto inválido.
    //
    // Retorna lista de conjuntos S (vectores de IDs 0-indexados).
    // -------------------------------------------------------------------------
    std::vector<std::vector<int>> findInvalidSets(const double* sol,
                                                  double threshold) const;
};

#endif // SUBTOUR_CUT_H
