#ifndef MECHANICS2D_H
#define MECHANICS2D_H

#include "physicsengine.h"
#include <QGraphicsScene>
#include <QAbstractGraphicsShapeItem>

class Mechanic2DWorld;

qreal vector_length(const QPointF & vector);


class Vector
{
public:
    Vector(const QString & name);
    Variable x,y;
};

class Shape2D
{
public:
    virtual ~Shape2D() {};
    QAbstractGraphicsShapeItem *item;
};

class ShapePolygon : public Shape2D
{
public:
    ShapePolygon(const QVector<QPointF> &points);
    QVector<QPointF> points;
    QVector<qreal> radius,angle,arc_length,inner_angle,adj_inner_angle;
};

class ShapeCircle : public Shape2D
{
public:
    ShapeCircle(qreal radius);
    qreal radius;
};

class Mechanic2DObject : public PhysicsObject
{
public:
    Mechanic2DObject(Mechanic2DWorld *world,const QString & name,Shape2D * shape);
    Vector p;
    Variable theta;
    Energy Ex,Ey,Ew;
    Mechanic2DWorld *world;
    QList<Vector*> pos_forces,neg_forces;
    QList<Variable*> pos_torque;
    Shape2D *shape;
};

class Graphics2D
{
public:
    Graphics2D(Mechanic2DWorld *world);
    virtual void update_graphics()=0;
};

class Mechanic2DWorld // : public PhysicsObject
{
public:
    Mechanic2DWorld(PhysicsEngine *engine,const QString & name);
    QGraphicsScene scene;
    QList<Mechanic2DObject*> objects;
    QList<Graphics2D*> graphics;
    PhysicsEngine *engine;

    virtual void update_graphics();
};

class MassObject : public Mechanic2DObject , public Graphics2D
{
public:
    MassObject(Mechanic2DWorld *world,const QString & name, Shape2D *shape=0);
    ~MassObject();
    Vector v;
    Variable w;
    qreal m,I;

    void calc_energy_diff();
    virtual void update_graphics();
    virtual void setup_equations();
    virtual void post_iteration();
};

class DownForce : public PhysicsObject
{
public:
    DownForce(MassObject *object,const QString & name);
    Vector f;
    Energy E;
    MassObject *object;
    qreal g;
    virtual void setup_equations();
    virtual void calc_energy_diff();
};

class SpringWallForce : public PhysicsObject
{
public:
    SpringWallForce(Mechanic2DObject *object,const QString & name);
    Vector f;
    Energy E;
    Mechanic2DObject *object;
    qreal k;
    virtual void setup_equations();
    virtual void calc_energy_diff();
};

class F1 : public PhysicsObject
{
public:
    F1(Mechanic2DObject *object1,Mechanic2DObject *object2,const QString & name);
    Mechanic2DObject *object1,*object2;
    Vector f;
    virtual void setup_equations();
    virtual void calc_energy_diff();
};

class ConstTorque : public PhysicsObject
{
public:
    ConstTorque(Mechanic2DObject *object,const QString & name,qreal torque);
    Variable t;
    Mechanic2DObject *object;
    qreal const_torque;
    virtual void setup_equations();
};

#endif // MECHANICS2D_H
