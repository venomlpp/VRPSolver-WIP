#ifndef SUBTOUR_CUT_H
#define SUBTOUR_CUT_H

#include <vector>
#include <coin/CglCutGenerator.hpp>
#include <coin/OsiSolverInterface.hpp>
#include <coin/OsiCuts.hpp>
#include <coin/OsiRowCut.hpp>
#include "Parser.h"

/*
 * Clase SubtourCutGenerator
 * Descripción: Generador de cortes DFJ (Dantzig-Fulkerson-Johnson) implementado
 * como un plugin CglCutGenerator para CBC. Evalúa dinámicamente las relajaciones LP 
 * en cada nodo del árbol Branch & Bound para detectar y penalizar subtours aislados 
 * y violaciones de capacidad mediante "lazy cuts".
 */
class SubtourCutGenerator : public CglCutGenerator {
public:
    SubtourCutGenerator(const Parser* parser, int numClients);

    void generateCuts(const OsiSolverInterface& si,
                      OsiCuts& cs,
                      const CglTreeInfo info = CglTreeInfo()) override;

    CglCutGenerator* clone() const override;

    ~SubtourCutGenerator() override = default;

private:
    const Parser* parserData;  
    int n;                     

    // Mapeo a índice lineal compatible con CbcSolver
    inline int getVarIndex(int i, int j) const { return i * n + j; }

    std::vector<std::vector<int>> findInvalidSets(const double* sol,
                                                  double threshold) const;
};

#endif // SUBTOUR_CUT_H