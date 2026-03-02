#ifndef CBC_SOLVER_H
#define CBC_SOLVER_H

#include <vector>
#include <chrono>
#include "Parser.h"
#include "Solution.h"
#include "Route.h"
#include "GreedyBuilder.h"
#include "VNS.h"
#include "SubtourCut.h"
#include <coin/CbcModel.hpp>
#include <coin/CbcEventHandler.hpp>

// ─────────────────────────────────────────────────────────────
// CbcProgressHandler
// Intercepta eventos de CBC para:
//   1. Reportar progreso cada 1000 nodos (similar al debug de B&B).
//   2. Aplicar VNS sobre cada nueva solución entera encontrada,
//      igual que hace B&B con lpGuidedConstruction+VNS.
//      Si VNS mejora la solución, la reinyecta en CBC como nueva
//      incumbente con UB más bajo, activando más poda.
// ─────────────────────────────────────────────────────────────
class CbcProgressHandler : public CbcEventHandler {
public:
    CbcProgressHandler(const Parser* parser, VNS* vns)
        : parserData(parser), vnsPtr(vns), lastReportNodes(0) {}

    CbcAction event(CbcEvent whichEvent) override {
        if (!model_) return noAction;

        int    nodes = model_->getNodeCount();
        double ub    = model_->getObjValue();
        double lb    = model_->getBestPossibleObjValue();
        double gap   = (ub < 1e49 && lb > -1e49) ?
                       100.0 * (ub - lb) / ub : 100.0;

        // Reporte periódico cada 1000 nodos
        if (nodes - lastReportNodes >= 1000) {
            lastReportNodes = nodes;
            std::cerr << "[CBC] Nodos: " << nodes
                      << " | LB: "  << lb
                      << " | UB: "  << (ub < 1e49 ? ub : -1.0)
                      << " | GAP: " << gap << "%"
                      << std::endl;
        }

        // Nueva solución entera encontrada: refinar con VNS
        if (whichEvent == solution && model_->bestSolution()) {
            double costBefore = ub;

            Solution cbcSol = convertToSolution(model_->bestSolution());
            if (cbcSol.isValid()) {
                Solution refined = vnsPtr->optimize(cbcSol, 20);
                std::cerr << "[CBC+VNS] Solucion " << (int)costBefore
                          << " -> " << refined.getTotalCost()
                          << " en nodo " << nodes << std::endl;

                // Si VNS mejoró, reinyectar en CBC como nueva incumbente
                if (refined.getTotalCost() < costBefore - 0.5) {
                    int n = parserData->getDimension();
                    int numVars = n * n + (n - 1);
                    std::vector<double> newSol(numVars, 0.0);
                    for (const auto& route : refined.getRoutes()) {
                        const auto& path = route.getPath();
                        for (size_t i = 0; i < path.size() - 1; ++i)
                            newSol[(path[i]-1)*n + (path[i+1]-1)] = 1.0;
                    }
                    model_->setBestSolution(newSol.data(), numVars,
                                            refined.getTotalCost());
                    model_->setCutoff(refined.getTotalCost() + 0.999);
                }
            }
        }

        return noAction;
    }

    CbcEventHandler* clone() const override {
        return new CbcProgressHandler(*this);
    }

private:
    const Parser* parserData;
    VNS*          vnsPtr;
    int           lastReportNodes;

    Solution convertToSolution(const double* sol) const {
        int n = parserData->getDimension();
        Solution s(parserData);
        for (int j = 1; j < n; ++j) {
            if (sol[0*n + j] > 0.5) {
                Route route(parserData->getCapacity(), parserData);
                int curr = j;
                while (curr != 0) {
                    route.addClient(curr + 1);
                    int next = -1;
                    for (int k = 0; k < n; ++k)
                        if (curr != k && sol[curr*n + k] > 0.5) { next = k; break; }
                    curr = next;
                    if (curr == -1) break;
                }
                s.addRoute(route);
            }
        }
        return s;
    }
};

// ─────────────────────────────────────────────────────────────
// CbcSolver
// ─────────────────────────────────────────────────────────────
class CbcSolver {
private:
    const Parser* parserData;
    int numClients;
    int numVariables;

    int getVarIndex(int i, int j) const;
    int getUIndex(int i) const;
    Solution convertToSolution(const double* solution) const;

public:
    explicit CbcSolver(const Parser* parser);

    // Resuelve el problema usando el motor exacto CBC de COIN-OR
    // con cortes DFJ, heurísticas internas y refinamiento VNS
    Solution solve(int numVehicles, double timeLimitSeconds = 120.0);
};

#endif // CBC_SOLVER_H
