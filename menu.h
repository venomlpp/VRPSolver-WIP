#ifndef MENU_H
#define MENU_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "Parser.h"
#include "Solution.h"
#include "GreedyBuilder.h"
#include "KOpt.h"
#include "VNS.h"
#include "BranchAndBound.h"
#include "CbcSolver.h"
#include "ALNS.h"

/*
 * Clase Menu
 * Descripción: Interfaz de usuario por consola para interactuar con 
 * los distintos algoritmos de resolución del problema CVRP. Administra 
 * el estado global de la ejecución y el manejo de instancias.
 */
class Menu {
private:
    std::unique_ptr<Parser> parserGlobal;
    Solution mejorSolucionGlobal;
    bool instanciaCargada;
    double tiempoLimiteGlobal;

    // ── Métodos auxiliares privados ───────────────────────────
    void mostrarEncabezado() const;
    void reportarTiempo(double tiempoTotal) const;
    void actualizarMejorSolucion(const Solution& solActual, const std::string& algoritmo);

    // ── Opciones del Menú ─────────────────────────────────────
    void cargarInstancia();
    void configurarTiempo();
    void ejecutar3OPT();
    void ejecutarBranchAndBound();
    void ejecutarMejorHeuristica();
    void ingresoManual();

public:
    Menu();
    void inicializar();
};

#endif // MENU_H