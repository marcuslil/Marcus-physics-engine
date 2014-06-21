#include "connections.h"

LoseConnection::LoseConnection(Mechanic2DObject *object1, Mechanic2DObject *object2, const QString &name)
    : PhysicsObject(object1->engine,name),Graphics2D(object1->world),f("f"),object1(object1),object2(object2)
{
    register_eq_variables(&f.x,&f.y);
    object1->pos_forces.append(&f);
    object2->neg_forces.append(&f);
    qreal vx=object2->p.x.init-object1->p.x.init;
    qreal vy=object2->p.y.init-object1->p.y.init;
    l=sqrt(vx*vx+vy*vy);

    line=new QGraphicsLineItem();
    object1->world->scene.addItem(line);
}

void LoseConnection::update_graphics(int history)
{
    line->setLine(object1->p.x.at(history),object1->p.y.at(history),object2->p.x.at(history),object2->p.y.at(history));
}


void LoseConnection::setup_equations()
{
    int row=f.x.nr;
    //(ax-bx)^2+(bx-by)^2=l^2;

    qreal vx=object1->p.x.curr()-object2->p.x.curr();
    qreal vy=object1->p.y.curr()-object2->p.y.curr();

    engine->A(row,object1->p.x.nr)=vx;
    engine->A(row,object2->p.x.nr)=-vx;
    engine->A(row,object1->p.y.nr)=vy;
    engine->A(row,object2->p.y.nr)=-vy;
    engine->B(row)=l*sqrt(vx*vx+vy*vy);

    row=f.y.nr;
    engine->A(row,f.x.nr)=-vy;
    engine->A(row,f.y.nr)=vx;
  //  engine->B(row)=1;
}

void LoseConnection::calc_energy_diff()
{
    engine->Em(object1->Ex.nr,object1->Ey.nr)=true;
    engine->Em(object1->Ex.nr,object2->Ex.nr)=true;
    engine->Em(object1->Ex.nr,object2->Ey.nr)=true;
}

FixedConnection::FixedConnection(Mechanic2DObject *object1, Mechanic2DObject *object2, const QString &name, bool fix_both)
    : PhysicsObject(object1->engine,name),Graphics2D(object1->world),f("f"),tau1("tau"),tau2("tau2"),object1(object1),object2(object2),fix_both(fix_both)
{
    register_eq_variables(&f.x,&f.y,&tau1);
    object1->pos_forces.append(&f);
    object2->neg_forces.append(&f);
    object1->pos_torque.append(&tau1);
    if (fix_both)
    {
        register_eq_variables(&tau2);
        object2->pos_torque.append(&tau2);
    }

    qreal vx=object2->p.x.init-object1->p.x.init;
    qreal vy=object2->p.y.init-object1->p.y.init;
    l=sqrt(vx*vx+vy*vy);
    if (l!=0.0)
        theta_1=atan2(vy,vx)-object1->theta.init;
    else
        theta_1=0.0;

    line=new QGraphicsLineItem();
    object1->world->scene.addItem(line);
}

void FixedConnection::update_graphics(int history)
{
    line->setLine(object1->p.x.at(history),object1->p.y.at(history),object2->p.x.at(history),object2->p.y.at(history));
}


void FixedConnection::setup_equations()
{
    int row;

    qreal theta_sum=object1->theta.curr()+theta_1,
            l_sin_theta=l*sin(theta_sum),
            l_cos_theta=l*cos(theta_sum);

    row=f.x.nr;
    engine->A(row,object1->p.x.nr)=-1.0;
    engine->A(row,object1->theta.nr)=l_sin_theta;
    engine->A(row,object2->p.x.nr)=1.0;
    engine->B(row)=l_cos_theta+l_sin_theta*object1->theta.curr();

    row=f.y.nr;
    engine->A(row,object1->p.y.nr)=-1.0;
    engine->A(row,object1->theta.nr)=-l_cos_theta;
    engine->A(row,object2->p.y.nr)=1.0;
    engine->B(row)=l_sin_theta-l_cos_theta*object1->theta.curr();

    row=tau1.nr;
    engine->A(row,f.x.nr)=l_sin_theta;
    engine->A(row,f.y.nr)=-l_cos_theta;
    engine->A(row,tau1.nr)=1.0;
    if (fix_both)
    {
        engine->A(row,tau2.nr)=1.0;

        row=tau2.nr;
        engine->A(row,object1->theta.nr)=1.0;
        engine->A(row,object2->theta.nr)=-1.0;
    }

}

void FixedConnection::calc_energy_diff()
{
    engine->Em(object1->Ex.nr,object1->Ey.nr)=true;
    engine->Em(object1->Ex.nr,object2->Ex.nr)=true;
    engine->Em(object1->Ex.nr,object2->Ey.nr)=true;
}


Fix::Fix(Mechanic2DObject *object, const QString &name, bool fix_torque)
    : PhysicsObject(object->engine,name),f("f"),tau("tau"),object(object),fix_torque(fix_torque)
{
    register_eq_variables(&f.x,&f.y);
    object->pos_forces.append(&f);
    if (fix_torque)
    {
        register_eq_variables(&tau);
        object->pos_torque.append(&tau);
    }

    x=object->p.x.init;
    y=object->p.y.init;
    theta=object->theta.init;
}

void Fix::setup_equations()
{
    const qreal weight=100.0;
    int row=f.x.nr;
    A(row,object->p.x.nr)=weight;
    B(row)=x*weight;

    row=f.y.nr;
    A(row,object->p.y.nr)=weight;
    B(row)=y*weight;

    if (fix_torque)
    {
        row=tau.nr;
        A(row,object->theta.nr)=weight;
        B(row)=theta*weight;
    }
}

