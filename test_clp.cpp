#include <iostream>
#include "Parser.h"
#include "ClpSolver.h"

using namespace std;

int main() {
    string filename = "A-n32-k5.vrp"; 
    Parser parser(filename);
    ClpSolver solver(&parser);
    
    // Usamos k=5 como dice el archivo
    solver.solveRelaxation(5); 
    
    return 0;
}