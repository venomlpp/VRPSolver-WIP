#include "menu.h"
#include <sstream>
#include <iomanip>
#include <limits>

using namespace std;
using chrono::steady_clock;
using chrono::duration;

// Constructor: Inicializa las variables
Menu::Menu() : parserGlobal(nullptr), instanciaCargada(false), tiempoLimiteGlobal(120.0) {
    // mejorSolucionGlobal se inicializa vacía
}

// ---------------------------------------------------------
// FUNCIONES AUXILIARES
// ---------------------------------------------------------

void Menu::mostrarEncabezado() const {
    cout << "\n========================================================" << endl;
    cout << "     EVALUACION 3: RUTEO DE VEHICULOS (CVRP)            " << endl;
    cout << "========================================================" << endl;
    if (instanciaCargada) {
        cout << "[Estado] Instancia: N=" << parserGlobal->getDimension() 
             << " | Q=" << parserGlobal->getCapacity() << endl;
    } else {
        cout << "[Estado] Ninguna instancia cargada." << endl;
    }
    cout << "[Estado] Tiempo Limite por Algoritmo: " << tiempoLimiteGlobal << " segundos." << endl;
    if (mejorSolucionGlobal.getTotalCost() > 0) {
        cout << "[Estado] Mejor Costo Encontrado (Global): " << mejorSolucionGlobal.getTotalCost() << endl;
    }
    cout << "--------------------------------------------------------" << endl;
}

void Menu::reportarTiempo(double tiempoTotal) const {
    cout << fixed << setprecision(3);
    cout << ">> Tiempo de ejecucion: " << tiempoTotal << " segundos." << endl;
}

void Menu::actualizarMejorSolucion(const Solution& solActual, const string& algoritmo) {
    // Como es minimización, verificamos si es menor (y mayor que 0 por si está vacía)
    if (solActual.getTotalCost() > 0 && 
       (mejorSolucionGlobal.getTotalCost() == 0 || solActual.getTotalCost() < mejorSolucionGlobal.getTotalCost())) {
        mejorSolucionGlobal = solActual;
        cout << ">> [!] NUEVA MEJOR SOLUCION GLOBAL ENCONTRADA POR " << algoritmo << ": " 
             << mejorSolucionGlobal.getTotalCost() << endl;
    } else {
        cout << ">> La solucion no mejora el optimo historico (" << mejorSolucionGlobal.getTotalCost() << ")." << endl;
    }
}

// ---------------------------------------------------------
// OPCIONES DEL MENÚ
// ---------------------------------------------------------

void Menu::cargarInstancia() {
    cout << "\nIngrese la ruta del archivo de la instancia (ej. sets/A-n32-k5.vrp): ";
    string ruta;
    cin >> ruta;
    
    try {
        parserGlobal = make_unique<Parser>(ruta);
        instanciaCargada = true;
        // Reseteamos la mejor solución histórica al cambiar de mapa
        mejorSolucionGlobal = Solution(parserGlobal.get()); 
        cout << ">> Archivo cargado correctamente." << endl;
    } catch (...) {
        cout << ">> Error al intentar cargar el archivo. Verifique la ruta." << endl;
        instanciaCargada = false;
    }
}

void Menu::configurarTiempo() {
    cout << "\nIngrese el nuevo tiempo limite en segundos (actual " << tiempoLimiteGlobal << "s): ";
    double nuevoTiempo;
    if (cin >> nuevoTiempo && nuevoTiempo > 0) {
        tiempoLimiteGlobal = nuevoTiempo;
        cout << ">> Tiempo limite actualizado a " << tiempoLimiteGlobal << " segundos." << endl;
    } else {
        cout << ">> Entrada invalida. Mantenido en " << tiempoLimiteGlobal << "s." << endl;
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }
}

void Menu::ejecutar3OPT() {
    cout << "\n--- Ejecutando Heuristica CW + 3-OPT ---" << endl;
    auto inicio = steady_clock::now();

    GreedyBuilder builder(parserGlobal.get());
    Solution cwSol = builder.buildSolution();
    
    KOpt kopt(parserGlobal.get());
    Solution koptSol = kopt.optimize(cwSol);

    double tiempo = duration<double>(steady_clock::now() - inicio).count();
    
    cout << "\n>> Costo inicial Clarke-Wright: " << cwSol.getTotalCost() << endl;
    cout << ">> Costo final tras 3-OPT: " << koptSol.getTotalCost() << endl;
    koptSol.print();
    reportarTiempo(tiempo);
    actualizarMejorSolucion(koptSol, "3-OPT");
}

void Menu::ejecutarBranchAndBound() {
    cout << "\n--- Ejecutando Branch & Bound + CLP ---" << endl;
    cout << "Usando limite de tiempo de " << tiempoLimiteGlobal << "s." << endl;
    auto inicio = steady_clock::now();

    // Warm Start exclusivo con 3-OPT (SIN VNS, como solicitaste)
    GreedyBuilder builder(parserGlobal.get());
    Solution cwSol = builder.buildSolution();
    KOpt kopt(parserGlobal.get());
    Solution warmStart = kopt.optimize(cwSol);

    cout << ">> Inyectando Warm Start (CW+3OPT): " << warmStart.getTotalCost() << endl;

    BranchAndBound bb(parserGlobal.get(), warmStart);
    // Por defecto usando Best-First, que dio mejores resultados en pruebas
    Solution bbSol = bb.solveBestFirst(tiempoLimiteGlobal);

    double tiempo = duration<double>(steady_clock::now() - inicio).count();

    cout << "\n>> Costo final B&B: " << bbSol.getTotalCost() << endl;
    bbSol.print();
    reportarTiempo(tiempo);
    actualizarMejorSolucion(bbSol, "Branch & Bound");
}

void Menu::ejecutarMejorHeuristica() {
    cout << "\n--- Ejecutando Mejor Heuristica (ALNS + LNS-MIP) ---" << endl;
    cout << "Usando limite de tiempo de " << tiempoLimiteGlobal << "s." << endl;
    auto inicio = steady_clock::now();

    // Warm Start VNS
    GreedyBuilder builder(parserGlobal.get());
    Solution cwSol = builder.buildSolution();
    VNS vns(parserGlobal.get());
    Solution vnsSol = vns.optimize(cwSol, 30); 

    // Motor Exacto (Repair)
    CbcSolver cbc(parserGlobal.get());
    
    // Metaheurística
    ALNS alns(parserGlobal.get(), &cbc);
    Solution alnsSol = alns.optimize(vnsSol, tiempoLimiteGlobal);

    // Pulido final
    Solution finalSol = vns.optimize(alnsSol, 30);

    double tiempo = duration<double>(steady_clock::now() - inicio).count();

    cout << "\n>> Costo final ALNS: " << finalSol.getTotalCost() << endl;
    finalSol.print();
    
    // Cálculo del GAP aproximado si tenemos un B&B que no terminó (opcional, aquí comparamos con CW)
    double gapCW = 100.0 * (cwSol.getTotalCost() - finalSol.getTotalCost()) / cwSol.getTotalCost();
    cout << ">> Mejora sobre inicial (CW): " << gapCW << "%" << endl;
    
    reportarTiempo(tiempo);
    actualizarMejorSolucion(finalSol, "ALNS+CBC");
}

void Menu::ingresoManual() {
    cout << "\n--- Ingreso de Rutas Manual ---" << endl;
    cout << "Ingrese el numero de vehiculos a utilizar: ";
    int k;
    if (!(cin >> k) || k <= 0) {
        cout << ">> Error: Numero de vehiculos invalido." << endl;
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        return;
    }
    
    // Limpiamos el buffer del enter
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    Solution solucionManual(parserGlobal.get());

    cout << "\nPara cada ruta, ingrese los ID de los clientes visitados separados por espacio." << endl;
    cout << "(NOTA: No es necesario ingresar la bodega '1' al principio o al final, se agrega automaticamente)" << endl;

    for (int i = 0; i < k; ++i) {
        cout << "Ruta del Vehiculo " << i + 1 << ": ";
        string linea;
        getline(cin, linea);
        
        stringstream ss(linea);
        int clienteId;
        Route ruta(parserGlobal->getCapacity(), parserGlobal.get());
        
        while (ss >> clienteId) {
            // Ignoramos si el usuario ingresó '1' accidentalmente
            if (clienteId != 1) {
                if (!ruta.addClient(clienteId)) {
                    cout << "  [Advertencia] Capacidad excedida al intentar agregar el cliente " << clienteId << endl;
                }
            }
        }
        solucionManual.addRoute(ruta);
    }

    cout << "\n>> Validando ruteo manual..." << endl;
    if (solucionManual.isValid()) {
        cout << ">> ESTADO: SOLUCION FACTIBLE Y VALIDA" << endl;
    } else {
        cout << ">> ESTADO: SOLUCION INVALIDA (Verifique capacidades, clientes faltantes o repetidos)" << endl;
    }

    cout << ">> Costo Total Evaluado (Z): " << solucionManual.getTotalCost() << endl;
    solucionManual.print();
    actualizarMejorSolucion(solucionManual, "Ingreso Manual");
}

// ---------------------------------------------------------
// BUCLE PRINCIPAL
// ---------------------------------------------------------

void Menu::inicializar() {
    string opcion;
    
    while (true) {
        mostrarEncabezado();
        cout << "1. Cargar archivo (Ingreso de grafo / instancia VRP)" << endl;
        cout << "2. Resolucion mediante Heuristica 3-OPT" << endl;
        cout << "3. Resolucion mediante Metodo Exacto Branch & Bound (CLP)" << endl;
        cout << "4. Resolucion mediante Mejor Heuristica (ALNS + CBC)" << endl;
        cout << "5. Ingreso de Ruteo Manual y calculo de costo" << endl;
        cout << "6. Ajustar Limite de Tiempo Global (Actual: " << tiempoLimiteGlobal << "s)" << endl;
        cout << "7. Salir" << endl;
        cout << "\nSeleccione una opcion: ";
        
        cin >> opcion;

        if (opcion == "1") {
            cargarInstancia();
        } 
        else if (opcion == "2") {
            if (!instanciaCargada) { cout << ">> Error: Debe cargar una instancia primero.\n"; continue; }
            ejecutar3OPT();
        } 
        else if (opcion == "3") {
            if (!instanciaCargada) { cout << ">> Error: Debe cargar una instancia primero.\n"; continue; }
            ejecutarBranchAndBound();
        } 
        else if (opcion == "4") {
            if (!instanciaCargada) { cout << ">> Error: Debe cargar una instancia primero.\n"; continue; }
            ejecutarMejorHeuristica();
        } 
        else if (opcion == "5") {
            if (!instanciaCargada) { cout << ">> Error: Debe cargar una instancia primero (para conocer Z y Q).\n"; continue; }
            ingresoManual();
        } 
        else if (opcion == "6") {
            configurarTiempo();
        } 
        else if (opcion == "7") {
            cout << ">> Saliendo del sistema CVRP. ¡Hasta luego!" << endl;
            break;
        } 
        else {
            cout << ">> Opcion no valida. Intente de nuevo." << endl;
        }
    }
}