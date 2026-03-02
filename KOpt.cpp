#include "KOpt.h"
#include <vector>
#include <algorithm> 
#include <iostream>

using namespace std;

/*
 * Descripción: Constructor de la heurística 3-OPT.
 * Entrada: Puntero constante a los datos parseados de la instancia.
 * Salida: Instancia inicializada.
 */
KOpt::KOpt(const Parser* parser) : parserData(parser) {}

/*
 * Descripción: Aplica la mejora 3-OPT a cada una de las rutas de una solución de manera independiente.
 * Entrada: Solución inicial a optimizar.
 * Salida: Objeto Solution con todas sus rutas en un mínimo local.
 */
Solution KOpt::optimize(const Solution& initialSolution) {
    Solution improvedSolution(parserData);
    
    for (const auto& route : initialSolution.getRoutes()) {
        Route optimized = optimizeRoute(route);
        improvedSolution.addRoute(optimized);
    }
    
    return improvedSolution;
}

/*
 * Descripción: Optimiza una sola ruta aplicando intercambios 3-OPT exhaustivos
 * hasta que no se encuentren más mejoras (mínimo local).
 * Entrada: Objeto Route a optimizar.
 * Salida: Nuevo objeto Route con la secuencia optimizada.
 */
Route KOpt::optimizeRoute(const Route& route) {
    vector<int> path = route.getPath();
    bool improvement = true;
    
    while (improvement) {
        improvement = false;
        int n = path.size();
        
        // Un 3-OPT requiere al menos 6 nodos para poder realizar 3 cortes válidos
        if (n < 6) break; 
        
        for (int i = 0; i <= n - 4 && !improvement; ++i) {
            for (int j = i + 1; j <= n - 3 && !improvement; ++j) {
                for (int k = j + 1; k <= n - 2 && !improvement; ++k) {
                    
                    int a = path[i],   b = path[i+1];
                    int c = path[j],   d = path[j+1];
                    int e = path[k],   f = path[k+1];
                    
                    int d0 = parserData->getDistance(a, b) + 
                             parserData->getDistance(c, d) + 
                             parserData->getDistance(e, f);
                    
                    // Evaluación O(1) de las 7 topologías de reconexión.
                    // Segmentos resultantes de los cortes: 
                    // S1 (inicio hasta a), S2 (b hasta c), S3 (d hasta e), S4 (f hasta fin)
                    
                    // Opción 1: Invertir S2 (2-OPT)
                    int d1 = parserData->getDistance(a, c) + parserData->getDistance(b, d) + parserData->getDistance(e, f);
                    // Opción 2: Invertir S3 (2-OPT)
                    int d2 = parserData->getDistance(a, b) + parserData->getDistance(c, e) + parserData->getDistance(d, f);
                    // Opción 3: Invertir S2 y S3
                    int d3 = parserData->getDistance(a, c) + parserData->getDistance(b, e) + parserData->getDistance(d, f);
                    // Opción 4: Intercambiar S2 y S3
                    int d4 = parserData->getDistance(a, d) + parserData->getDistance(e, b) + parserData->getDistance(c, f);
                    // Opción 5: Intercambiar S2 y S3, invirtiendo S2
                    int d5 = parserData->getDistance(a, d) + parserData->getDistance(e, c) + parserData->getDistance(b, f);
                    // Opción 6: Intercambiar S2 y S3, invirtiendo S3
                    int d6 = parserData->getDistance(a, e) + parserData->getDistance(d, b) + parserData->getDistance(c, f);
                    // Opción 7: Intercambiar S2 y S3, invirtiendo ambos
                    int d7 = parserData->getDistance(a, e) + parserData->getDistance(d, c) + parserData->getDistance(b, f);
                    
                    int best_delta = 0;
                    int best_opt = 0;
                    
                    if (d1 - d0 < best_delta) { best_delta = d1 - d0; best_opt = 1; }
                    if (d2 - d0 < best_delta) { best_delta = d2 - d0; best_opt = 2; }
                    if (d3 - d0 < best_delta) { best_delta = d3 - d0; best_opt = 3; }
                    if (d4 - d0 < best_delta) { best_delta = d4 - d0; best_opt = 4; }
                    if (d5 - d0 < best_delta) { best_delta = d5 - d0; best_opt = 5; }
                    if (d6 - d0 < best_delta) { best_delta = d6 - d0; best_opt = 6; }
                    if (d7 - d0 < best_delta) { best_delta = d7 - d0; best_opt = 7; }
                    
                    // Aplicar la mejor reconexión encontrada
                    if (best_opt > 0) {
                        improvement = true;
                        
                        vector<int> S1(path.begin(), path.begin() + i + 1);
                        vector<int> S2(path.begin() + i + 1, path.begin() + j + 1);
                        vector<int> S3(path.begin() + j + 1, path.begin() + k + 1);
                        vector<int> S4(path.begin() + k + 1, path.end());
                        
                        vector<int> S2_rev = S2; std::reverse(S2_rev.begin(), S2_rev.end());
                        vector<int> S3_rev = S3; std::reverse(S3_rev.begin(), S3_rev.end());
                        
                        vector<int> newPath = S1; 
                        
                        if (best_opt == 1) {
                            newPath.insert(newPath.end(), S2_rev.begin(), S2_rev.end());
                            newPath.insert(newPath.end(), S3.begin(), S3.end());
                        } else if (best_opt == 2) {
                            newPath.insert(newPath.end(), S2.begin(), S2.end());
                            newPath.insert(newPath.end(), S3_rev.begin(), S3_rev.end());
                        } else if (best_opt == 3) {
                            newPath.insert(newPath.end(), S2_rev.begin(), S2_rev.end());
                            newPath.insert(newPath.end(), S3_rev.begin(), S3_rev.end());
                        } else if (best_opt == 4) {
                            newPath.insert(newPath.end(), S3.begin(), S3.end());
                            newPath.insert(newPath.end(), S2.begin(), S2.end());
                        } else if (best_opt == 5) {
                            newPath.insert(newPath.end(), S3.begin(), S3.end());
                            newPath.insert(newPath.end(), S2_rev.begin(), S2_rev.end());
                        } else if (best_opt == 6) {
                            newPath.insert(newPath.end(), S3_rev.begin(), S3_rev.end());
                            newPath.insert(newPath.end(), S2.begin(), S2.end());
                        } else if (best_opt == 7) {
                            newPath.insert(newPath.end(), S3_rev.begin(), S3_rev.end());
                            newPath.insert(newPath.end(), S2_rev.begin(), S2_rev.end());
                        }
                        
                        newPath.insert(newPath.end(), S4.begin(), S4.end());
                        
                        path = newPath; 
                    }
                }
            }
        }
    }
    
    Route optimizedRoute(parserData->getCapacity(), parserData);
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        optimizedRoute.addClient(path[i]);
    }
    
    return optimizedRoute;
}

/*
 * Descripción: Destructor de la clase.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
KOpt::~KOpt() {}