#ifndef CLIENT_H
#define CLIENT_H

/*
 * Clase Client
 * Descripción: Representa un nodo o cliente dentro del problema CVRP, almacenando
 * su identificador, coordenadas espaciales y demanda de carga.
 */
class Client {
private:
    int id;
    double x;
    double y;
    int demand;

public:
    Client();
    Client(int id, double x, double y, int demand);

    int getId() const;
    double getX() const;
    double getY() const;
    int getDemand() const;

    ~Client();
};

#endif // CLIENT_H