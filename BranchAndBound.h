#ifndef BRANCH_AND_BOUND_H
#define BRANCH_AND_BOUND_H

#include <vector>
#include <queue>  // Para priority_queue
#include <stack>  // Lo dejamos importado para cuando hagamos la comparacion
#include "Parser.h"
#include "Solution.h"

// Cabecera principal del solver continuo de COIN-OR (CLP)
#include <coin/ClpSimplex.hpp> 

// Estructura que representa un nodo en el arbol de exploracion
struct BBNode {
    int level;                           // Profundidad en el arbol
    double lowerBound;                   // El costo continuo (Z) que CLP calculo para este nodo
    
    std::vector<double> lowerBounds;     // Limites inferiores (0 o 1) para cada variable x_ij
    std::vector<double> upperBounds;     // Limites superiores (0 o 1) para cada variable x_ij

    // Sobrecarga del operador < para que priority_queue funcione como un Min-Heap.
    // Al retornar "mayor que", la cola sacara primero el nodo con el menor lowerBound.
    bool operator<(const BBNode& other) const {
        return lowerBound > other.lowerBound;
    }
};

class BranchAndBound {
private:
    const Parser* parserData;
    double globalUpperBound;             // Nuestra Cota Superior (Record a batir, ej: 17386)
    Solution bestSolution;               // La mejor solucion entera encontrada
    
    int numClients;                      // N (Bodega + Clientes)
    int numVariables;                    // Total de variables x_ij (N * N)

    // --- Metodos de Formulacion Matematica ---
    
    // Construye el modelo CVRP relajado inicial (Funcion Objetivo y Restricciones base)
    void buildBaseModel(ClpSimplex& model);
    
    // Mapea coordenadas 2D (origen i, destino j) a un indice 1D para el arreglo de variables de CLP
    int getVarIndex(int i, int j) const;

    // --- Metodos de Logica del Arbol ---

    // Encuentra la variable fraccionaria mas cercana a 0.5 para ramificar
    int getMostFractionalVariable(const double* solution) const;
    
    // Verifica si TODAS las variables de la solucion son enteras (0.0 o 1.0)
    bool isIntegerSolution(const double* solution) const;
    
    // Transforma el arreglo de 0s y 1s de CLP en nuestra clase Solution de C++
    Solution convertToSolution(const double* solution) const;

public:
    // Constructor: Requiere el problema y la solucion inicial (UB) del 3-OPT
    BranchAndBound(const Parser* parser, const Solution& initialSolution);

    // Metodo principal de resolucion usando Cola de Prioridad (Best-First Search)
    Solution solveBestFirst();

    // Metodo secundario (lo implementaremos despues) usando Pila (Depth-First Search)
    // Solution solveDepthFirst();

    ~BranchAndBound();
};

#endif // BRANCH_AND_BOUND_H