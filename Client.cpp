#include "Client.h"

/*
 * Descripción: Constructor por defecto. Inicializa un cliente vacío o nodo base (bodega).
 * Entrada: Ninguna.
 * Salida: Instancia de Client con valores en cero.
 */
Client::Client() : id(0), x(0.0), y(0.0), demand(0) {}

/*
 * Descripción: Constructor parametrizado para inicializar un cliente con sus datos.
 * Entrada: ID numérico, coordenada X, coordenada Y, demanda de carga.
 * Salida: Instancia de Client con los atributos asignados.
 */
Client::Client(int id, double x, double y, int demand) 
    : id(id), x(x), y(y), demand(demand) {}

/*
 * Descripción: Obtiene el ID del cliente.
 * Entrada: Ninguna.
 * Salida: Valor entero con el ID.
 */
int Client::getId() const { return id; }

/*
 * Descripción: Obtiene la coordenada espacial X del cliente.
 * Entrada: Ninguna.
 * Salida: Valor decimal con la posición X.
 */
double Client::getX() const { return x; }

/*
 * Descripción: Obtiene la coordenada espacial Y del cliente.
 * Entrada: Ninguna.
 * Salida: Valor decimal con la posición Y.
 */
double Client::getY() const { return y; }

/*
 * Descripción: Obtiene la demanda de carga requerida por el cliente.
 * Entrada: Ninguna.
 * Salida: Valor entero con la demanda.
 */
int Client::getDemand() const { return demand; }

/*
 * Descripción: Destructor de la clase.
 * Entrada: Ninguna.
 * Salida: Ninguna.
 */
Client::~Client() {}