#include "Client.h"

// Constructor por defecto
Client::Client() : id(0), x(0.0), y(0.0), demand(0) {}

// Constructor parametrizado
Client::Client(int id, double x, double y, int demand) 
    : id(id), x(x), y(y), demand(demand) {}

// Getters
int Client::getId() const { return id; }
double Client::getX() const { return x; }
double Client::getY() const { return y; }
int Client::getDemand() const { return demand; }

// Destructor
Client::~Client() {}