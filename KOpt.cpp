#include "KOpt.h"
#include <vector>
#include <algorithm> // Para std::reverse
#include <iostream>

using namespace std;

// Constructor
KOpt::KOpt(const Parser* parser) : parserData(parser) {}

// Método principal que itera sobre todas las rutas
Solution KOpt::optimize(const Solution& initialSolution) {
    Solution improvedSolution(parserData);
    
    // Optimizamos vehículo por vehículo
    for (const auto& route : initialSolution.getRoutes()) {
        Route optimized = optimizeRoute(route);
        improvedSolution.addRoute(optimized);
    }
    
    return improvedSolution;
}

// El motor 3-OPT: Optimiza una sola ruta hasta llegar al mínimo local
Route KOpt::optimizeRoute(const Route& route) {
    vector<int> path = route.getPath();
    bool improvement = true;
    
    while (improvement) {
        improvement = false;
        int n = path.size();
        
        // Un 3-OPT real requiere cortar 3 aristas, lo que significa que la ruta 
        // debe tener al menos 6 nodos (Ej: Bodega -> A -> B -> C -> D -> Bodega)
        if (n < 6) break; 
        
        // Iteramos buscando 3 cortes: i, j, k
        for (int i = 0; i <= n - 4 && !improvement; ++i) {
            for (int j = i + 1; j <= n - 3 && !improvement; ++j) {
                for (int k = j + 1; k <= n - 2 && !improvement; ++k) {
                    
                    // Nodos en los extremos de las 3 aristas cortadas
                    int a = path[i],   b = path[i+1];
                    int c = path[j],   d = path[j+1];
                    int e = path[k],   f = path[k+1];
                    
                    // Costo base (las 3 aristas que vamos a eliminar)
                    int d0 = parserData->getDistance(a, b) + 
                             parserData->getDistance(c, d) + 
                             parserData->getDistance(e, f);
                    
                    // Evaluamos en O(1) las 7 formas de reconectar los 4 segmentos resultantes
                    // Segmentos: S1(inicio hasta a), S2(b hasta c), S3(d hasta e), S4(f hasta fin)
                    
                    // Opción 1: Invertir S2 (Equivalente a 2-OPT)
                    int d1 = parserData->getDistance(a, c) + parserData->getDistance(b, d) + parserData->getDistance(e, f);
                    // Opción 2: Invertir S3 (Equivalente a 2-OPT)
                    int d2 = parserData->getDistance(a, b) + parserData->getDistance(c, e) + parserData->getDistance(d, f);
                    // Opción 3: Invertir S2 y S3
                    int d3 = parserData->getDistance(a, c) + parserData->getDistance(b, e) + parserData->getDistance(d, f);
                    // Opción 4: Intercambiar S2 y S3 de lugar (Puro 3-OPT)
                    int d4 = parserData->getDistance(a, d) + parserData->getDistance(e, b) + parserData->getDistance(c, f);
                    // Opción 5: Intercambiar S2 y S3, invirtiendo S2
                    int d5 = parserData->getDistance(a, d) + parserData->getDistance(e, c) + parserData->getDistance(b, f);
                    // Opción 6: Intercambiar S2 y S3, invirtiendo S3
                    int d6 = parserData->getDistance(a, e) + parserData->getDistance(d, b) + parserData->getDistance(c, f);
                    // Opción 7: Intercambiar S2 y S3, invirtiendo ambos
                    int d7 = parserData->getDistance(a, e) + parserData->getDistance(d, c) + parserData->getDistance(b, f);
                    
                    // Buscamos la mejor mejora (First-Improvement / Best-Improvement local)
                    int best_delta = 0;
                    int best_opt = 0;
                    
                    if (d1 - d0 < best_delta) { best_delta = d1 - d0; best_opt = 1; }
                    if (d2 - d0 < best_delta) { best_delta = d2 - d0; best_opt = 2; }
                    if (d3 - d0 < best_delta) { best_delta = d3 - d0; best_opt = 3; }
                    if (d4 - d0 < best_delta) { best_delta = d4 - d0; best_opt = 4; }
                    if (d5 - d0 < best_delta) { best_delta = d5 - d0; best_opt = 5; }
                    if (d6 - d0 < best_delta) { best_delta = d6 - d0; best_opt = 6; }
                    if (d7 - d0 < best_delta) { best_delta = d7 - d0; best_opt = 7; }
                    
                    // Si encontramos una reconexión que acorte la distancia, la aplicamos físicamente
                    if (best_opt > 0) {
                        improvement = true;
                        
                        // Extraemos los bloques
                        vector<int> S1(path.begin(), path.begin() + i + 1);
                        vector<int> S2(path.begin() + i + 1, path.begin() + j + 1);
                        vector<int> S3(path.begin() + j + 1, path.begin() + k + 1);
                        vector<int> S4(path.begin() + k + 1, path.end());
                        
                        vector<int> S2_rev = S2; std::reverse(S2_rev.begin(), S2_rev.end());
                        vector<int> S3_rev = S3; std::reverse(S3_rev.begin(), S3_rev.end());
                        
                        vector<int> newPath = S1; // Empezamos a ensamblar
                        
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
                        
                        // Sobrescribimos la ruta actual y el while reiniciará la búsqueda
                        path = newPath; 
                    }
                }
            }
        }
    }
    
    // Una vez que el ciclo while termina, significa que esta ruta está en su mínimo local perfecto.
    // Creamos un nuevo objeto Route para validar y empaquetar de forma segura.
    Route optimizedRoute(parserData->getCapacity(), parserData);
    
    // Saltamos el índice 0 y el índice final (que son la bodega 1), 
    // porque el constructor de Route ya los inicializa por defecto.
    for (size_t i = 1; i < path.size() - 1; ++i) {
        optimizedRoute.addClient(path[i]);
    }
    
    return optimizedRoute;
}

KOpt::~KOpt() {}