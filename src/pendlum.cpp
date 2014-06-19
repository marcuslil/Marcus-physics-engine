#include "pendlum.h"
#include <QDebug>

SimplePendlum::SimplePendlum(MassObject *object, const QString &name, qreal x, qreal y)
    : PhysicsObject(object->engine,name),Graphics2D(object->world),f("f"),x(x),y(y),object(object)
{
    register_variables(&f.x,&f.y);
    object->pos_forces.append(&f);
    line=new QGraphicsLineItem();
    object->world->scene.addItem(line);
    qreal vx=object->p.x.curr-x;
    qreal vy=object->p.y.curr-y;
    l=sqrt(vx*vx+vy*vy);
}

void SimplePendlum::update_graphics()
{
    line->setLine(x,y,object->p.x.curr,object->p.y.curr);
}

void SimplePendlum::setup_equations()
{
    qreal vx=object->p.x.curr-x;
    qreal vy=object->p.y.curr-y;

    qreal scale=l/sqrt(vx*vx+vy*vy);
    vx=vx*scale;
    vy=vy*scale;

    int row=f.x.nr;
    engine->A(row,object->p.x.nr)=vx;
    engine->A(row,object->p.y.nr)=vy;
    engine->B(row)=(vx*(vx+x)+vy*(vy+y));

    row=f.y.nr;
    engine->A(row,f.x.nr)=vy;
    engine->A(row,f.y.nr)=-vx;
}

void SimplePendlum::calc_energy_diff()
{
    engine->Em(object->Ex.nr,object->Ey.nr)=true;
}


CompoundPendlum::CompoundPendlum(MassObject *object, const QString &name,qreal x,qreal y)
    : PhysicsObject(object->engine,name),Graphics2D(object->world),f("f"),x(x),y(y),t("torque"),object(object)
{
    register_variables(&f.x,&f.y,&t);
    object->pos_forces.append(&f);
    object->pos_torque.append(&t);

    qreal vx=object->p.x.curr-x;
    qreal vy=object->p.y.curr-y;
    l=sqrt(vx*vx+vy*vy);
    theta=atan2(vy,vx)-object->theta.curr;

    line=new QGraphicsLineItem();
    object->world->scene.addItem(line);
}

void CompoundPendlum::update_graphics()
{
    line->setLine(x,y,object->p.x.curr,object->p.y.curr);
}

void CompoundPendlum::setup_equations()
{
    qreal theta_sum=object->theta.curr+theta,
            l_sin_theta=l*sin(theta_sum),
            l_cos_theta=l*cos(theta_sum);

    // px=fpx+l*cos(th+thf)-l*sin(th+thf)*dth ->
    // px=fpx+l*cos(th+thf)-l*sin(th+thf)*th+l*sin(th+thf)*th_old ->
    // px+l*sin(th+thf)*th=fpx+l*cos(th+thf)+l*sin(th+thf)*th_old

    int row=f.x.nr;
    engine->A(row,object->p.x.nr)=1;
    engine->A(row,object->theta.nr)=l_sin_theta;
    engine->B(row)=x+l_cos_theta+l_sin_theta*object->theta.curr;

    // py=fpy+l*sin(th+thf)+l*cos(th+thf)*dth ->
    // py=fpy+l*sin(th+thf)+l*cos(th+thf)*th-l*cos(th+thf)*th_old ->
    // py-l*cos(th+thf)*th=fpy+l*sin(th+thf)-l*cos(th+thf)*th_old

    row=f.y.nr;
    engine->A(row,object->p.y.nr)=1;
    engine->A(row,object->theta.nr)=-l_cos_theta;
    engine->B(row)=y+l_sin_theta-l_cos_theta*object->theta.curr;

    row=t.nr;
    engine->A(row,f.x.nr)=l_sin_theta; //vy/l_curr;
    engine->A(row,f.y.nr)=-l_cos_theta; //-vx/l_curr;
    engine->A(row,t.nr)=-1.0;

}

void CompoundPendlum::calc_energy_diff()
{
    engine->Em(object->Ex.nr,object->Ey.nr)=true;
    engine->Em(object->Ex.nr,object->Ew.nr)=true;
}

MathPendlum::MathPendlum(Mechanic2DWorld *world, const QString &name, qreal x, qreal y, qreal T, qreal m)
 : PhysicsObject(world->engine,name),Graphics2D(world),x(x),y(y),T(T),m(m)
{
    rect=new QGraphicsRectItem();
    rect->setRect(-1,-1,2,2);
    rect->setBrush(Qt::red);
    world->scene.addItem(rect);
    update_graphics();
}

void MathPendlum::update_graphics()
{
    rect->setPos(x+cos(2.0*M_PI*engine->t/T)*m,y);
}

void MathPendlum::calc_energy_diff()
{

}

void MathPendlum::setup_equations()
{

}

StringPendlum::StringPendlum(MassObject *object, const QString &name, qreal x, qreal y)
    : PhysicsObject(object->engine,name),Graphics2D(object->world),at_edge("at edge",true),loss("loss"),f("f"),x(x),y(y),object(object)
{
    register_variables(&f.x,&f.y);
    register_states(&at_edge);
    register_energies(&loss);
    object->pos_forces.append(&f);
    line=new QGraphicsLineItem();
    object->world->scene.addItem(line);
    qreal vx=object->p.x.curr-x;
    qreal vy=object->p.y.curr-y;
    l=sqrt(vx*vx+vy*vy);
}

void StringPendlum::update_graphics()
{
    line->setLine(x,y,object->p.x.curr,object->p.y.curr);
}

void StringPendlum::setup_equations()
{
    qreal vx=object->p.x.curr-x;
    qreal vy=object->p.y.curr-y;

    qreal curr_l=sqrt(vx*vx+vy*vy);
    qreal scale=l/curr_l;
    vx=vx*scale;
    vy=vy*scale;

    qreal F=f.x.curr*vx+f.y.curr*vy;

    if (!at_edge.curr.value<bool>() && curr_l>=l*1) at_edge.curr.setValue<bool>(true);
    else if (at_edge.curr.value<bool>() && F>0) at_edge.curr.setValue<bool>(false);

    if (at_edge.curr.value<bool>())
    {
        int row=f.x.nr;
        engine->A(row,object->p.x.nr)=vx;
        engine->A(row,object->p.y.nr)=vy;
        engine->B(row)=(vx*(vx+x)+vy*(vy+y));

        row=f.y.nr;
        engine->A(row,f.x.nr)=vy;
        engine->A(row,f.y.nr)=-vx;
    }
    else
    {
        engine->A(f.x.nr,f.x.nr)=1;
        engine->A(f.y.nr,f.y.nr)=1;
    }

}

void StringPendlum::calc_energy_diff()
{
    engine->Em(object->Ex.nr,object->Ey.nr)=true;
    if (!at_edge.last_t.value<bool>() && at_edge.curr.value<bool>())
    {
        qreal dx=object->p.x.curr-x;
        qreal dy=object->p.y.curr-y;
        qreal v=object->v.x.last_t*dx/l+object->v.y.last_t*dy/l;
        loss.delta=v*v*object->m/2.0;
    }
    else
        loss.delta=0;
}

