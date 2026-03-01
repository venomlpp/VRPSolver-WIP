#include <iostream>
#include "Parser.h"
#include "Route.h"

using namespace std;

void printRouteState(const Route& route) {
    cout << "Ruta actual: [ ";
    for (int nodeId : route.getPath()) {
        cout << nodeId << " ";
    }
    cout << "]" << endl;
    cout << "Carga: " << route.getCurrentLoad() << endl;
    cout << "Costo total: " << route.getTotalCost() << endl;
    cout << "Es valida (sin subtours/excesos)? " << (route.isValid() ? "Si" : "No") << endl;
    cout << "---------------------------------" << endl;
}

int main() {
    cout << "--- Iniciando Test de Route ---" << endl;
    
    // 1. Cargar el parser con la instancia real
    string filename = "X-n115-k10.vrp";
    Parser parser(filename);
    
    cout << "Capacidad maxima de la instancia: " << parser.getCapacity() << endl;
    cout << "---------------------------------" << endl;

    // 2. Crear una ruta vacia
    // Le pasamos la capacidad máxima y la dirección de memoria del parser (&parser)
    Route miRuta(parser.getCapacity(), &parser);
    
    cout << "Estado inicial de la ruta (Debe ser [1 1] con costo/carga 0):" << endl;
    printRouteState(miRuta);

    // 3. Agregar el Cliente 2 (Demanda: 86)
    cout << "Intentando agregar Cliente 2 (Demanda: 86)..." << endl;
    bool exito2 = miRuta.addClient(2);
    cout << "Exito? " << (exito2 ? "Si" : "No") << endl;
    printRouteState(miRuta);

    // 4. Intentar agregar el Cliente 3 (Demanda: 98) - DEBE FALLAR
    cout << "Intentando agregar Cliente 3 (Demanda: 98)..." << endl;
    bool exito3 = miRuta.addClient(3);
    cout << "Exito? " << (exito3 ? "Si" : "No") << " <- (Deberia ser No, supera 169)" << endl;
    printRouteState(miRuta);

    // 5. Agregar el Cliente 4 (Demanda: 89) - DEBE FALLAR TAMBIEN (86 + 89 = 175)
    cout << "Intentando agregar Cliente 4 (Demanda: 89)..." << endl;
    bool exito4 = miRuta.addClient(4);
    cout << "Exito? " << (exito4 ? "Si" : "No") << " <- (Deberia ser No, supera 169)" << endl;
    printRouteState(miRuta);

    // 6. Agregar el Cliente 6 (Demanda: 74) - DEBE FUNCIONAR (86 + 74 = 160 <= 169)
    cout << "Intentando agregar Cliente 6 (Demanda: 74)..." << endl;
    bool exito6 = miRuta.addClient(6);
    cout << "Exito? " << (exito6 ? "Si" : "No") << " <- (Deberia ser Si)" << endl;
    printRouteState(miRuta);

    cout << "--- Test superado exitosamente ---" << endl;

    return 0;
}