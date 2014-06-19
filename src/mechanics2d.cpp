#include "mechanics2d.h"

#include <QGraphicsPathItem>
#include <QDebug>

qreal vector_length(const QPointF & vector)
{
    return sqrt(vector.x()*vector.x()+vector.y()*vector.y());
}


Mechanic2DWorld::Mechanic2DWorld(PhysicsEngine *engine, const QString & /*name*/)
//    : PhysicsObject(engine,name)
{
    this->engine=engine;
}

void Mechanic2DWorld::update_graphics()
{
    for (int g=0;g!=graphics.size();g++)
        graphics[g]->update_graphics();
}

Graphics2D::Graphics2D(Mechanic2DWorld *world)
{
    world->graphics.append(this);
}

Vector::Vector(const QString &name)
    : x(name+".x"),y(name+".y")
{
}

ShapePolygon::ShapePolygon(const QVector<QPointF> &points)
    : points(points)
{
    QPainterPath p;
    QVector<QPointF> points2(points);
    angle.resize(points.size());
    radius.resize(points.size());
    arc_length.resize(points.size());
    for (int p=0;p!=points.size();p++)
    {
        radius[p]=vector_length(points.at(p));
        angle[p]=atan2(points.at(p).y(),points.at(p).x());
        arc_length[p]=vector_length(points.at(p)-points.at((p+1)%points.size()));
    }
    inner_angle.resize(points.size());
    adj_inner_angle.resize(points.size());
    for (int p=0;p!=points.size();p++)
    {
        int pp1=(p+1)%points.size();
        arc_length[p]=vector_length(points.at(p)-points.at((p+1)%points.size()));
        inner_angle[p]=acos((radius.at(p)*radius.at(p)+arc_length.at(p)*arc_length.at(p)-radius.at(pp1)*radius.at(pp1))/2.0/radius.at(p)/arc_length.at(p));
        adj_inner_angle[p]=M_PI-inner_angle.at(p)-(angle.at(pp1)-angle.at(p)) ;
    }
    points2.append(points.at(0));
    p.addPolygon(QPolygonF(points2));
    QVector<QPointF> p2;
    p2.append(QPointF(0,0));
    p2.append(QPointF(1,0));
    p.addPolygon(p2);
    item=new QGraphicsPathItem(p);
}

ShapeCircle::ShapeCircle(qreal radius)
    : radius(radius)
{
    item=new QGraphicsEllipseItem(0,0,radius,radius);
}

Mechanic2DObject::Mechanic2DObject(Mechanic2DWorld *world, const QString &name, Shape2D *shape)
    : PhysicsObject(world->engine,name),p("p"),theta("theta"),Ex("Ex"),Ey("Ey"),Ew("Ew"),world(world),shape(shape)
{
    world->objects.append(this);
    register_variables(&p.x,&p.y,&theta);
    register_energies(&Ex,&Ey,&Ew);
    if (this->shape==0)
    {
        QVector<QPointF> p;
       /* p.append(QPointF(-1,-1));
        p.append(QPointF(-1,1));
        p.append(QPointF(1,1));
        p.append(QPointF(1,-1));*/
        p.append(QPointF(-1,-1));
        p.append(QPointF(1,-1));
        p.append(QPointF(1,1));
        p.append(QPointF(-1,1));


        this->shape=new ShapePolygon(p);
        this->shape->item->setBrush(Qt::red);
    }
    world->scene.addItem(this->shape->item);
}

MassObject::MassObject(Mechanic2DWorld *world, const QString &name,Shape2D *shape)
    : Mechanic2DObject(world,name,shape),Graphics2D(world),v("v"),w("w")
{
    register_variables(&v.x,&v.y,&w); // p and theta already registered in Mechanic2DObject
    //register_energies(&Ex,&Ey,&Ew); energies alrady registered
    p.x=0;
    p.y=0;
    v.x=0;
    v.y=0;
    theta=0;
    w=0;
    m=1;
    I=1;
    update_graphics();
}

void MassObject::setup_equations()
{
    int row;
//    dpx/dt=vx; -> px_next-vx*dt=px
    row=p.x.nr;
    engine->A(row,p.x.nr)=1;
    engine->A(row,v.x.nr)=-engine->k*engine->delta_t;
    engine->B(row)=p.x.last_t+v.x.last_t*engine->delta_t*(1.0-engine->k);

//    dpy/dt=vy; -> py_next-vy*dt=py
    row=p.y.nr;
    engine->A(row,p.y.nr)=1;
    engine->A(row,v.y.nr)=-engine->k*engine->delta_t;
    engine->B(row)=p.y.last_t+v.y.last_t*engine->delta_t*(1.0-engine->k);


    //   dvx/dt=1/m*sum(Fx);
    row=v.x.nr;
    engine->A(row,v.x.nr)=1;
    for(int f=0;f!=pos_forces.size();f++)
    {
        engine->A(row,pos_forces.at(f)->x.nr)=-engine->delta_t/m*engine->k;
        engine->B(row)+=pos_forces.at(f)->x.last_t*(1.0-engine->k)/m*engine->delta_t;

    }
    for(int f=0;f!=neg_forces.size();f++)
    {
        engine->A(row,neg_forces.at(f)->x.nr)=engine->delta_t/m*engine->k;
        engine->B(row)+=-neg_forces.at(f)->x.last_t*(1.0-engine->k)/m*engine->delta_t;
    }
    engine->B(row)+=v.x.last_t;

    //   dvy/dt=1/m*sum(Fy);
    row=v.y.nr;
    engine->A(row,v.y.nr)=1;
    for(int f=0;f!=pos_forces.size();f++)
    {
        engine->A(row,pos_forces.at(f)->y.nr)=-engine->delta_t/m*engine->k;
        engine->B(row)+=pos_forces.at(f)->y.last_t*(1.0-engine->k)/m*engine->delta_t;
    }
    for(int f=0;f!=neg_forces.size();f++)
    {
        engine->A(row,neg_forces.at(f)->y.nr)=engine->delta_t/m*engine->k;
        engine->B(row)+=-neg_forces.at(f)->y.last_t*(1.0-engine->k)/m*engine->delta_t;
    }
   engine->B(row)+=v.y.last_t;

    // dth/dt=w;
    row=theta.nr;
    engine->A(row,theta.nr)=1;
    engine->A(row,w.nr)=-engine->delta_t*engine->k;
    engine->B(row)=theta.last_t+w.last_t*engine->delta_t*(1.0-engine->k);

    // dw/dt=1/i*sum(Torque)
    row=w.nr;
    engine->A(row,w.nr)=1;
//    engine->Bv[row]=1;
    for(int f=0;f!=pos_torque.size();f++)
    {
        engine->A(row,pos_torque.at(f)->nr)=-engine->delta_t/I*engine->k;
        engine->B(row)+=pos_torque.at(f)->last_t/I*(1.0-engine->k)*engine->delta_t;
    }
    engine->B(row)+=w.last_t;
}

void MassObject::update_graphics()
{
    shape->item->setPos(p.x.curr,p.y.curr);
    shape->item->setRotation(theta.curr*180.0/M_PI);
}

void MassObject::calc_energy_diff()
{
    Ex.delta=m*(v.x.curr*v.x.curr-v.x.last_t*v.x.last_t)/2.0;
    Ey.delta=m*(v.y.curr*v.y.curr-v.y.last_t*v.y.last_t)/2.0;
    Ew.delta=I*(w.curr*w.curr-w.last_t*w.last_t)/2.0;
}

void MassObject::post_iteration()
{
  //  while(theta.curr>M_PI) theta.curr-=2.0*M_PI;
  //  while(theta.curr<-M_PI) theta.curr+=2.0*M_PI;
}

MassObject::~MassObject()
{
   world->scene.removeItem(shape->item);
}

PhysicsObject::PhysicsObject(PhysicsEngine *engine,const QString & name)
    : engine(engine),name(name)
{
    if (name.isNull())
        this->name=QString("Object %1").arg(engine->objects.size());
    engine->objects.append(this);
}

DownForce::DownForce(MassObject *object, const QString &name)
    : PhysicsObject(object->engine,name),f("f"),E("E"),object(object)
{
    register_variables(&f.x,&f.y);
    register_energies(&E);
    object->pos_forces.append(&f);
    f.x=0;
    g=9.82;
    f.y=object->m*g;
}


void DownForce::setup_equations()
{
    engine->A(f.x.nr,f.x.nr)=1;
    engine->A(f.y.nr,f.y.nr)=1;
    engine->B(f.y.nr)=object->m*g;
}

void DownForce::calc_energy_diff()
{
    E.delta=g*object->m*(object->p.y.last_t-object->p.y.curr);
    engine->Em(E.nr,object->Ey.nr)=true;
}

SpringWallForce::SpringWallForce(Mechanic2DObject *object, const QString &name)
    :PhysicsObject(object->engine,name),f("f"),E("E"),object(object)
{
    register_variables(&f.x,&f.y);
    register_energies(&E);
    object->pos_forces.append(&f);
    f.x=0;
    f.y=0;
    k=1;
}


void SpringWallForce::setup_equations()
{
    engine->A(f.x.nr,f.x.nr)=1;
    engine->A(f.y.nr,f.y.nr)=1;
    if (object->p.y.curr>10)
    {
        engine->A(f.y.nr,object->p.y.nr)=k; //*(object->p.y.val-10);
        engine->B(f.y.nr)=k*10;
    //    engine->Bc(f.y.nr)=-10*k;
    }
}

void SpringWallForce::calc_energy_diff()
{
    qreal curr=0,last_t=0;
    if (object->p.y.curr>10)
        curr=object->p.y.curr-10;
    if (object->p.y.last_t>10)
        last_t=object->p.y.last_t-10;
    E.delta=k*(curr*curr-last_t*last_t)/2;
}


F1::F1(Mechanic2DObject *object1, Mechanic2DObject *object2, const QString &name)
    : PhysicsObject(object1->engine,name),object1(object1),object2(object2),f("f")
{
    register_variables(&f.x,&f.y);
    object1->pos_forces.append(&f);
    f.x=0;
    f.y=0;
}

void F1::setup_equations()
{
    int row=f.x.nr;
    //(ax-bx)^2+(bx-by)^2=l^2;

    qreal vx=object1->p.x.curr-object2->p.x.curr;
    qreal vy=object1->p.y.curr-object2->p.y.curr;
    qreal l=sqrt(vx*vx+vy*vy);
    vx/=l;
    vy/=l;

    engine->A(row,f.x.nr)=1.0;
    engine->B(row)=-vy;

    row=f.y.nr;
    engine->A(row,f.y.nr)=1.0;
    engine->B(row)=vx;
}

void F1::calc_energy_diff()
{
}

ConstTorque::ConstTorque(Mechanic2DObject *object, const QString &name, qreal torque)
    : PhysicsObject(object->engine,name),t("t"),object(object),const_torque(torque)
{
    register_variables(&t);
    object->pos_torque.append(&t);
}

void ConstTorque::setup_equations()
{
    A(t.nr,t.nr)=1.0;
    B(t.nr)=const_torque;
}

