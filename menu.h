#ifndef MENU_H
#define MENU_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

// Incluimos los headers del core del proyecto
#include "Parser.h"
#include "Solution.h"
#include "GreedyBuilder.h"
#include "KOpt.h"
#include "VNS.h"
#include "BranchAndBound.h"
#include "CbcSolver.h"
#include "ALNS.h"

class Menu {
private:
    std::unique_ptr<Parser> parserGlobal;
    Solution mejorSolucionGlobal;
    bool instanciaCargada;
    double tiempoLimiteGlobal;

    // Métodos auxiliares privados
    void mostrarEncabezado() const;
    void reportarTiempo(double tiempoTotal) const;
    void actualizarMejorSolucion(const Solution& solActual, const std::string& algoritmo);

    // Opciones del Menú
    void cargarInstancia();
    void configurarTiempo();
    void ejecutar3OPT();
    void ejecutarBranchAndBound();
    void ejecutarMejorHeuristica();
    void ingresoManual();

public:
    Menu();              // Constructor
    void inicializar();  // El bucle principal del programa
};

#endif // MENU_H