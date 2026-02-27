#ifndef CLIENT_H
#define CLIENT_H

class Client {
private:
    int id;
    double x;
    double y;
    int demand;

public:
    // Constructores
    Client();
    Client(int id, double x, double y, int demand);

    // Getters
    int getId() const;
    double getX() const;
    double getY() const;
    int getDemand() const;

    // Destructor
    ~Client();
};

#endif // CLIENT_H