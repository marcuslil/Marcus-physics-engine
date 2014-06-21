#ifndef CONNECTIONS_H
#define CONNECTIONS_H

#include "mechanics2d.h"

class LoseConnection : public PhysicsObject, public Graphics2D
{
public:
    LoseConnection(Mechanic2DObject *object1,Mechanic2DObject *object2,const QString & name);
    Vector f;
    qreal l;
    Mechanic2DObject *object1,*object2;
    QGraphicsLineItem *line;
    virtual void setup_equations();
    virtual void calc_energy_diff();
    virtual void update_graphics(int history);
};

class FixedConnection : public PhysicsObject, public Graphics2D
{
public:
    FixedConnection(Mechanic2DObject *object1,Mechanic2DObject *object2,const QString & name,bool fix_both = true);
    Vector f;
    RealVariable tau1,tau2;
    qreal l,theta_1,theta_2;
    Mechanic2DObject *object1,*object2;
    QGraphicsLineItem *line;
    virtual void setup_equations();
    virtual void calc_energy_diff();
    virtual void update_graphics(int history);
private:
    bool fix_both;
};

class Fix : public PhysicsObject
{
public:
    Fix(Mechanic2DObject *object,const QString & name,bool fix_torque = true);
    Vector f;
    RealVariable tau;
    Mechanic2DObject *object;
    qreal x,y,theta;
    virtual void setup_equations();
private:
    bool fix_torque;

};

#endif // CONNECTIONS_H
