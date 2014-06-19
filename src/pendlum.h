#ifndef PENDLUM_H
#define PENDLUM_H

#include <QGraphicsLineItem>
#include "mechanics2d.h"

class SimplePendlum : public PhysicsObject, public Graphics2D
{
public:
    SimplePendlum(MassObject *object,const QString & name,qreal x=0.0,qreal y=0.0);
private:
    Vector f;
    qreal l,x,y;
    MassObject *object;
    QGraphicsLineItem *line;
    virtual void setup_equations();
    virtual void calc_energy_diff();
    virtual void update_graphics();
};

class CompoundPendlum : public PhysicsObject, public Graphics2D
{
public:
    CompoundPendlum(MassObject *object,const QString & name, qreal x, qreal y);
    virtual void setup_equations();
    virtual void calc_energy_diff();
    virtual void update_graphics();
private:
    Vector f;
    qreal x,y;
    Variable t;
    MassObject *object;
    QGraphicsLineItem *line;
    qreal l,theta;

};

class MathPendlum  : public PhysicsObject, public Graphics2D
{
public:
    MathPendlum(Mechanic2DWorld *world, const QString &name, qreal x, qreal y, qreal T, qreal m);
private:
    qreal x,y,T,m;
    virtual void setup_equations();
    virtual void calc_energy_diff();
    virtual void update_graphics();
    QGraphicsRectItem *rect;
};

class StringPendlum : public PhysicsObject, public Graphics2D
{
public:
    StringPendlum(MassObject *object,const QString & name,qreal x=0.0,qreal y=0.0);
private:
    State at_edge;
    Energy loss;
    Vector f;
    qreal l,x,y;
    MassObject *object;
    QGraphicsLineItem *line;
    virtual void setup_equations();
    virtual void calc_energy_diff();
    virtual void update_graphics();
};

#endif // PENDLUM_H
