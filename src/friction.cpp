#include "friction.h"
#include <QTransform>
#include <QDebug>

Friction::Friction(Mechanic2DObject *object1, Mechanic2DObject *object2, const QString &name)
    : PhysicsObject(object1->engine,name),Graphics2D(object1->world),f("f"),tau1("tau1"),tau2("tau2"),state("state"),prevstate("prevstate"),index1("index1"),index2("index2"),angle("angle"),angle2("angle2"),length("length"),object1(object1),object2(object2)
{
    register_eq_variables(&f.x,&f.y,&tau1,&tau2);
    register_variables(&state,&prevstate,&angle,&angle2,&length,&index1,&index2);
    object1->pos_forces.append(&f);
    object2->neg_forces.append(&f);
    object1->pos_torque.append(&tau1);
    object2->pos_torque.append(&tau2);
    state.init=(int)Free;
    prevstate.init=(int)Free;
    item=new QGraphicsEllipseItem(-0.1,-0.1,0.2,0.2);
    item->setVisible(false);
    item2=new QGraphicsEllipseItem(-0.1,-0.1,0.2,0.2);
    item2->setVisible(false);
    object1->world->scene.addItem(item);
    object1->world->scene.addItem(item2);
}

bool point_cossed_line(QPointF l1_bef,QPointF l2_bef,QPointF p_bef,QPointF l1_aft,QPointF l2_aft,QPointF p_aft,qreal & x)
{
    QPointF v=l1_bef-l2_bef;
    qreal length=sqrt(v.x()*v.x()+v.y()*v.y());

    l2_bef-=l1_bef;
    p_bef-=l1_bef;
    p_bef=QPointF(l2_bef.x()*p_bef.x()+l2_bef.y()*p_bef.y(),-l2_bef.y()*p_bef.x()+l2_bef.x()*p_bef.y())/length;

    l2_aft-=l1_aft;
    p_aft-=l1_aft;
    p_aft=QPointF(l2_aft.x()*p_aft.x()+l2_aft.y()*p_aft.y(),-l2_aft.y()*p_aft.x()+l2_aft.x()*p_aft.y())/length;

    if (p_bef.y()<0.00 && p_aft.y()>=-0.0)
    {
        x=(p_bef.y()*p_aft.x()-p_bef.x()*p_aft.y())/(p_bef.y()-p_aft.y());
        if (x>=0 && x<=length)
        {
//            qDebug() << "hit at" << x;
            return true;
        }
    }
    return false;

}


bool check_coll(Mechanic2DObject *object1,Mechanic2DObject *object2,int & line1,int & point1,qreal & dist,bool filter=false)
{
    ShapePolygon *p1=dynamic_cast<ShapePolygon*> (object1->shape);
    ShapePolygon *p2=dynamic_cast<ShapePolygon*> (object2->shape);

    int line_first=0,line_last=p1->points.size()-1;
    int point_first=0,point_last=p2->points.size()-1;
    if (filter)
    {
        line_first=line_last=line1;
        point_first=point_last=point1;
    }

    for (int line=line_first;line<=line_last;line++)
    {
        int l_next=line==p1->points.size()-1?0:line+1;
        const qreal & r1=p1->radius.at(line),& r2=p1->radius.at(l_next);
        const qreal & a1=p1->angle.at(line), & a2=p1->angle.at(l_next);

#ifdef USE_PREV_T
        QLineF line_last_t(object1->p.x.prev_t()+r1*cos(a1+object1->theta.prev_t()),
                           object1->p.y.prev_t()+r1*sin(a1+object1->theta.prev_t()),
                           object1->p.x.prev_t()+r2*cos(a2+object1->theta.prev_t()),
                           object1->p.y.prev_t()+r2*sin(a2+object1->theta.prev_t()));
#else
        QLineF line_last_t(object1->p.x.prev()+r1*cos(a1+object1->theta.prev()),
                           object1->p.y.prev()+r1*sin(a1+object1->theta.prev()),
                           object1->p.x.prev()+r2*cos(a2+object1->theta.prev()),
                           object1->p.y.prev()+r2*sin(a2+object1->theta.prev()));
#endif


        QLineF   line_curr(object1->p.x.curr()+r1*cos(a1+object1->theta.curr()),
                           object1->p.y.curr()+r1*sin(a1+object1->theta.curr()),
                           object1->p.x.curr()+r2*cos(a2+object1->theta.curr()),
                           object1->p.y.curr()+r2*sin(a2+object1->theta.curr()));

        for (int point=point_first;point<=point_last;point++)
        {
            const qreal & r=p2->radius.at(point),& a=p2->angle.at(point);

#ifdef USE_PREV_T
            QPointF point_last_t(object2->p.x.prev_t()+r*cos(a+object2->theta.prev_t()),
                                 object2->p.y.prev_t()+r*sin(a+object2->theta.prev_t()));
#else
            QPointF point_last_t(object2->p.x.prev()+r*cos(a+object2->theta.prev()),
                                 object2->p.y.prev()+r*sin(a+object2->theta.prev()));
#endif

            QPointF   point_curr(object2->p.x.curr()+r*cos(a+object2->theta.curr()),
                                 object2->p.y.curr()+r*sin(a+object2->theta.curr()));


      //      qDebug() << "testing: point nr " << point_2 << "(" << point << ") on " <<object2->name << "if crossed line nr" << line_1 << "(" << line << ") on " << object1->name;
       //     qDebug() << line_start_curr << line_stop_curr << point_curr;
       //     qDebug() << line_start_last_t << line_stop_last_t << point_last_t;
            //static int hitnr=0;
            if (point_cossed_line(line_last_t.p1(),line_last_t.p2(),point_last_t,line_curr.p1(),line_curr.p2(),point_curr,dist))
            {
                //qDebug() << hitnr++<< "hit: point nr " << point << " on " << object2->name << " crossed line nr " << line << "on " << object1->name;
                line1=line;
                point1=point;
                return true;
            }
           // qDebug() << dist;
        }


    }
    return false;

}

bool rs2(QPointF l1_aft,QPointF l2_aft,QPointF p_aft)
{
    QPointF v=l1_aft-l2_aft;
    qreal length=sqrt(v.x()*v.x()+v.y()*v.y());

    l2_aft-=l1_aft;
    p_aft-=l1_aft;
    p_aft=QPointF(l2_aft.x()*p_aft.x()+l2_aft.y()*p_aft.y(),-l2_aft.y()*p_aft.x()+l2_aft.x()*p_aft.y())/length;

    if (p_aft.y()>=0)
    {
            return true;
    }
    return false;

}


bool right_side(Mechanic2DObject *object1,Mechanic2DObject *object2,int line,int point)
{
    ShapePolygon *p1=dynamic_cast<ShapePolygon*> (object1->shape);
    ShapePolygon *p2=dynamic_cast<ShapePolygon*> (object2->shape);



    int l_next=line==p1->points.size()-1?0:line+1;
    Q_ASSERT(line>-1 && line<p1->points.size());
    Q_ASSERT(point>-1 && point<p2->points.size());
    const qreal & r1=p1->radius.at(line),& r2=p1->radius.at(l_next);
    const qreal & a1=p1->angle.at(line), & a2=p1->angle.at(l_next);

    QLineF   line_curr(object1->p.x.curr()+r1*cos(a1+object1->theta.curr()),
                       object1->p.y.curr()+r1*sin(a1+object1->theta.curr()),
                       object1->p.x.curr()+r2*cos(a2+object1->theta.curr()),
                       object1->p.y.curr()+r2*sin(a2+object1->theta.curr()));

    const qreal & r=p2->radius.at(point),& a=p2->angle.at(point);

    QPointF   point_curr(object2->p.x.curr()+r*cos(a+object2->theta.curr()),
                         object2->p.y.curr()+r*sin(a+object2->theta.curr()));


    //static int hitnr=0;
    if (rs2(line_curr.p1(),line_curr.p2(),point_curr))
    {
        //qDebug() << hitnr++<< "hit: point nr " << point << " on " << object2->name << " wrong side off " << line << "on " << object1->name;
        return true;
    }

    return false;

}


bool inside(Mechanic2DObject *object1,Mechanic2DObject *object2,int &p)
{
    ShapePolygon *p1=dynamic_cast<ShapePolygon*> (object1->shape);
    ShapePolygon *p2=dynamic_cast<ShapePolygon*> (object2->shape);
    for (p=0;p!=p1->points.size();p++)
    {
        bool all_inside=true;
        for(int l=0;l!=p2->points.size() && all_inside;l++)
            all_inside &= right_side(object2,object1,l,p);
        if (all_inside) return true;
    }
    return false;
}


void Friction::setup_equations()
{
    int row;
    if (state.curr()==Free)
    {
        A(f.x.nr,f.x.nr)=1.0;
        A(f.y.nr,f.y.nr)=1.0;
        A(tau1.nr,tau1.nr)=1.0;
        A(tau2.nr,tau2.nr)=1.0;
    }
    else if (state.curr()==Line1_to_Point2_fixed || state.curr()==Point1_to_Line2_fixed)
    {
        qreal r1sin,r1cos,r2sin,r2cos;

        if (state.curr()==Line1_to_Point2_fixed)
        {
            ShapePolygon *p=dynamic_cast<ShapePolygon*>(object2->shape);
            r1sin=length.curr()*sin(angle.curr()+object1->theta.curr());
            r1cos=length.curr()*cos(angle.curr()+object1->theta.curr());
            r2sin=p->radius.at(index2.curr())*sin(p->angle.at(index2.curr())+object2->theta.curr());
            r2cos=p->radius.at(index2.curr())*cos(p->angle.at(index2.curr())+object2->theta.curr());
        }
        else
        {
            ShapePolygon *p=dynamic_cast<ShapePolygon*>(object1->shape);
            r1sin=p->radius.at(index2.curr())*sin(p->angle.at(index2.curr())+object1->theta.curr());
            r1cos=p->radius.at(index2.curr())*cos(p->angle.at(index2.curr())+object1->theta.curr());
            r2sin=length.curr()*sin(angle.curr()+object2->theta.curr());
            r2cos=length.curr()*cos(angle.curr()+object2->theta.curr());
        }
        //eq1
        row=f.x.nr;
        A(row,object1->p.x.nr)=1.0;
        A(row,object1->theta.nr)=-r1sin;
        A(row,object2->p.x.nr)=-1.0;
        A(row,object2->theta.nr)=r2sin;
        B(row)=-r1cos-r1sin*object1->theta.curr()+r2cos+r2sin*object2->theta.curr();

        //eq2
        row=f.y.nr;
        A(row,object1->p.y.nr)=1.0;
        A(row,object1->theta.nr)=r1cos;
        A(row,object2->p.y.nr)=-1.0;
        A(row,object2->theta.nr)=-r2cos;
        B(row)=-r1sin+r1cos*object1->theta.curr()+r2sin-r2cos*object2->theta.curr();

        //eq3
        row=tau1.nr;
        A(row,f.x.nr)=-r1sin;
        A(row,f.y.nr)=r1cos;
        A(row,tau1.nr)=-1.0;

        //eq4
        row=tau2.nr;
        A(row,f.x.nr)=-r2sin;
        A(row,f.y.nr)=r2cos;
        A(row,tau2.nr)=1.0;
    }
    else if (state.curr()==Line1_to_Line2_fixed)
    {
        qreal theta_sum=object1->theta.curr()+angle.curr(),
                l_sin_theta=length.curr()*sin(theta_sum),
                l_cos_theta=length.curr()*cos(theta_sum);

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
        engine->A(row,tau2.nr)=1.0;

//        -theta1+theta2-angle2=0;
        int l1=0,l2=0;
        while(-object1->theta.curr()+object2->theta.curr()-angle2.curr()>M_PI && l1<200)
        {
            angle2.curr()=angle2.curr()+2.0*M_PI; //,qDebug() << Q_FUNC_INFO << "Adjust angle +2pi";
            l1++;
            if (l1>100)
            {
                //qDebug() << Q_FUNC_INFO << l1 <<object1->theta.curr() << object2->theta.curr() << angle2.curr();
               // return;

//                exit(0);
            }
        }
        while(-object1->theta.curr()+object2->theta.curr()-angle2.curr()<-M_PI && l2<200)
        {
            angle2.curr()=angle2.curr()-2.0*M_PI; //,qDebug() << Q_FUNC_INFO << "Adjust angle -2pi";
            l2++;
            if (l2>100)
            {
                //qDebug() << Q_FUNC_INFO << l1<< l2<<object1->theta.curr() << object1->theta.prev_t() << object2->theta.curr() << angle2.curr();
  //              exit(0);
            }
        }
        row=tau2.nr;
        engine->A(row,object1->theta.nr)=-1.0;
        engine->A(row,object2->theta.nr)=1.0;
        B(row)=angle2.curr();
    }
}

void Friction::calc_energy_diff()
{

}

qreal sgn(qreal a)
{
    if (a<0.0) return -1.0;
    if (a>0.0) return 1.0;
    return 0.0;
}

void Friction::post_iteration()
{
    int _line,_point;
    qreal dist;
    int prev_state=prevstate.curr();
    prevstate.curr()=state.curr();
    if (state.curr()==Free)
    {
        if (abs(object1->p.x.curr()-object2->p.x.curr()) > object1->shape->max_center_dist + object2->shape->max_center_dist) return;
        if (abs(object1->p.y.curr()-object2->p.y.curr()) > object1->shape->max_center_dist + object2->shape->max_center_dist) return;
        bool swap=false;
        if (prev_state==Line1_to_Point2_fixed && right_side(object1,object2,  index1.curr(),index2.curr()))
        {
            state.curr()=Line1_to_Point2_fixed;
            return;
        }
        else if (prev_state==Point1_to_Line2_fixed && right_side(object2,object1,  index1.curr(),index2.curr()))
        {
            state.curr()=Point1_to_Line2_fixed;
            return;
        }
        else if (check_coll(object1,object2,_line,_point,dist) ||  (swap=check_coll(object2,object1,_line,_point,dist)))
        {
            ShapePolygon *p1;
            if (!swap)
            {
                state.curr()=Line1_to_Point2_fixed;
                p1=dynamic_cast<ShapePolygon*> (object1->shape);
            }
            else
            {
                state.curr()=Point1_to_Line2_fixed;
                p1=dynamic_cast<ShapePolygon*> (object2->shape);
            }

            index1.curr()=_line;
            index2.curr()=_point;

            QPointF point_on_line=p1->points.at(_line);
            QPointF dir=p1->points.at((_line+1)%p1->points.size())-point_on_line;
            dir*=dist/vector_length(dir);
            point_on_line+=dir;
            length.curr()=vector_length(point_on_line);
            angle.curr()=atan2(point_on_line.y(),point_on_line.x());
            return;
        }
    }

    if (state.curr()==Line1_to_Point2_fixed || state.curr()==Point1_to_Line2_fixed)
    {
        ShapePolygon *p1=dynamic_cast<ShapePolygon*> (object1->shape);
        ShapePolygon *p2=dynamic_cast<ShapePolygon*> (object2->shape);

        int i1=index1.curr(); // line
        int i2=index2.curr(); // point
        int i1max=state.curr()==Line1_to_Point2_fixed?p1->points.size():p2->points.size();
        int i2max=state.curr()==Line1_to_Point2_fixed?p2->points.size():p1->points.size();
        Q_ASSERT(i1<i1max);
        Q_ASSERT(i2<i2max);
        int i1p1=(i1+1)%i1max;
//        int i1m1=(i1-1+i1max)%i1max;
        int i2m1=(i2-1+i2max)%i2max;
        int i2p1=(i2+1)%i2max;

//        if (state.curr()==Line1_to_Point2_fixed && (check_coll(object2,object1,i2m1,i1p1,dist,true) || check_coll(object1,object2,i1,i2m1,dist,true) ))
//        if (state.curr()==Line1_to_Point2_fixed && (check_coll(object2,object1,i2m1,i1p1,dist,true) || check_coll(object1,object2,i1,i2m1,dist,true) || right_side(object2,object1,i2m1,i1p1) || right_side(object1,object2,i1,i2m1)))
        if (state.curr()==Line1_to_Point2_fixed && (right_side(object2,object1,i2m1,i1p1) || right_side(object1,object2,i1,i2m1)))
        {
            //qDebug() << "second hit 1";

            const qreal & theta1=p1->angle.at(i1);
            const qreal & theta2=angle.curr();
            const qreal & theta3=p2->adj_inner_angle.at(i2m1);
            const qreal & theta4=p1->inner_angle.at(i1);
            const qreal & theta5=p2->angle.at(i2);

            qreal alpha1=theta2-theta1;
            qreal alpha2=M_PI-theta4-alpha1;
            qreal alpha3=theta2+alpha2-theta3;
            qreal alpha4=M_PI-theta5+alpha3;

            qreal vx=length.curr()*cos(theta2)+p2->radius.at(i2)*cos(alpha3);
            qreal vy=length.curr()*sin(theta2)+p2->radius.at(i2)*sin(alpha3);

            state.curr()=Line1_to_Line2_fixed;
            length.curr()=vector_length(QPointF(vx,vy));
            angle.curr()=atan2(vy,vx);
            angle2.curr()=alpha4;
            index1.curr()=i1;
            index2.curr()=i2m1;
        }
//        else if (state.curr()==Point1_to_Line2_fixed && (check_coll(object1,object2,i2m1,i1p1,dist,true) || check_coll(object2,object1,i1,i2m1,dist,true) ))
//        else if (state.curr()==Point1_to_Line2_fixed && (check_coll(object1,object2,i2m1,i1p1,dist,true) || check_coll(object2,object1,i1,i2m1,dist,true) || right_side(object1,object2,i2m1,i1p1) || right_side(object2,object1,i1,i2m1)))
        else if (state.curr()==Point1_to_Line2_fixed && (right_side(object1,object2,i2m1,i1p1) || right_side(object2,object1,i1,i2m1)))
        {
            //qDebug() << "second hit 1 222";
            const qreal & theta1=p2->angle.at(i1);
            const qreal & theta2=angle.curr();
            const qreal & theta3=p1->adj_inner_angle.at(i2m1);
            const qreal & theta4=p2->inner_angle.at(i1);
            const qreal & theta5=p1->angle.at(i2);

            qreal alpha1=theta2-theta1;
            qreal alpha2=M_PI-theta4-alpha1;
            qreal alpha3=theta2+alpha2-theta3;
           // qreal alpha4=M_PI-theta5+alpha3;
            qreal alpha7=theta5-alpha3+theta2;
            qreal alpha8=alpha7+1.0*M_PI-theta2;

            qreal vx=length.curr()*cos(alpha7)+p1->points.at(i2).x();
            qreal vy=length.curr()*sin(alpha7)+p1->points.at(i2).y();

            state.curr()=Line1_to_Line2_fixed;
            length.curr()=vector_length(QPointF(vx,vy));
            angle.curr()=atan2(vy,vx);
            angle2.curr()=alpha8;
            index1.curr()=i2m1;
            index2.curr()=i1;

        }
//        else if (state.curr()==Line1_to_Point2_fixed && (check_coll(object1,object2,i1,i2p1,dist,true) || check_coll(object2,object1,i2,i1,dist,true)))
//        else if (state.curr()==Line1_to_Point2_fixed && (check_coll(object1,object2,i1,i2p1,dist,true) || check_coll(object2,object1,i2,i1,dist,true) || right_side(object1,object2,i1,i2p1) || right_side(object2,object1,i2,i1)))
        else if (state.curr()==Line1_to_Point2_fixed && (right_side(object1,object2,i1,i2p1) || right_side(object2,object1,i2,i1)))
        {
            //qDebug() << "second hit2";

            const qreal & theta1=p1->angle.at(i1);
            const qreal & theta2=angle.curr();
            const qreal & theta3=p2->inner_angle.at(i2);
            const qreal & theta4=p1->inner_angle.at(i1);
            const qreal & theta5=p2->angle.at(i2);

            qreal alpha1=theta2-theta1;
            qreal alpha2=M_PI-theta4-alpha1;
            qreal alpha3=theta2-M_PI+(alpha2+theta3);
            qreal alpha4=M_PI-theta5+alpha3;

            qreal vx=length.curr()*cos(theta2)+p2->radius.at(i2)*cos(alpha3);
            qreal vy=length.curr()*sin(theta2)+p2->radius.at(i2)*sin(alpha3);

            state.curr()=Line1_to_Line2_fixed;
            length.curr()=vector_length(QPointF(vx,vy));
            angle.curr()=atan2(vy,vx);
            angle2.curr()=alpha4;
            index1.curr()=i1;
            index2.curr()=i2;
        }
//        else if (state.curr()==Point1_to_Line2_fixed && (check_coll(object2,object1,i1,i2p1,dist,true) || check_coll(object1,object2,i2,i1,dist,true)))
//        else if (state.curr()==Point1_to_Line2_fixed && (check_coll(object2,object1,i1,i2p1,dist,true) || check_coll(object1,object2,i2,i1,dist,true) || right_side(object2,object1,i1,i2p1) || right_side(object1,object2,i2,i1)))
        else if (state.curr()==Point1_to_Line2_fixed && (right_side(object2,object1,i1,i2p1) || right_side(object1,object2,i2,i1)))
        {
            //qDebug() << "second hit2 222";

            const qreal & theta1=p2->angle.at(i1);
            const qreal theta2=angle.curr(); //-angle2.curr();
            const qreal & theta3=p1->inner_angle.at(i2);
            const qreal & theta4=p2->inner_angle.at(i1);
            const qreal & theta5=p1->angle.at(i2);

            qreal alpha1=theta2-theta1;
            qreal alpha2=M_PI-theta4-alpha1;
            //qreal alpha3=theta2-M_PI+(alpha2+theta3);
            qreal alpha5=theta5+M_PI-(theta3+alpha2);
//            qreal alpha4=M_PI-theta5+alpha3;
            qreal alpha6=alpha5+M_PI-theta2;

//            qDebug() << theta2 << alpha3 << theta1;

            qreal vx=length.curr()*cos(alpha5)+p1->points.at(i2).x();
            qreal vy=length.curr()*sin(alpha5)+p1->points.at(i2).y();


  //          qDebug() << vx << vy;
            state.curr()=Line1_to_Line2_fixed;

//            vx=object2->p.x.curr()-object1->p.x.curr();
//            vy=object2->p.y.curr()-object1->p.y.curr();

            length.curr()=vector_length(QPointF(vx,vy));
            angle.curr()=atan2(vy,vx); //+M_PI/2.0;
            angle2.curr()=alpha6; //+M_PI/2.0; //alpha4;
 //           angle2.curr()=object2->theta.curr()-object1->theta.curr()+angle.curr();
            index1.curr()=i2;
            index2.curr()=i1;
        }
//        else if (state.curr()==Line1_to_Point2_fixed && (right_side(object2,object1,i2m1,i1p1) || right_side(object1,object2,i1,i2m1) )) qDebug() << "!1";
//       else if (state.curr()==Point1_to_Line2_fixed && (right_side(object1,object2,i2m1,i1p1) || right_side(object2,object1,i1,i2m1)))  qDebug() << "!2";
//        else if (state.curr()==Line1_to_Point2_fixed && (right_side(object1,object2,i1,i2p1) || right_side(object2,object1,i2,i1)))  qDebug() << "!3";
//        else if (state.curr()==Point1_to_Line2_fixed && (right_side(object2,object1,i1,i2p1) || right_side(object1,object2,i2,i1)))  qDebug() << "!4";


        if (state.curr()==Line1_to_Point2_fixed)
        {
            const qreal & theta1=p1->angle.at(i1);
            const qreal alpha2=theta1+M_PI-p1->inner_angle.at(i1);

            const qreal a3=alpha2+object1->theta.curr();

          //  const qreal Fh=f.x.curr()*cos(a3)+f.y.curr()*sin(a3);
            const qreal Fv=f.x.curr()*cos(a3+M_PI/2.0)+f.y.curr()*sin(a3+M_PI/2.0);

            if (Fv<-0.0)
            {
                //qDebug() << "goto free1" << Fv;
                state.curr()=Free;
                return;
            }
        }

        if (state.curr()==Point1_to_Line2_fixed)
        {
            const qreal & theta1=p2->angle.at(i1);
            const qreal alpha2=theta1+M_PI-p2->inner_angle.at(i1);

            const qreal a3=alpha2+object2->theta.curr();

            //const qreal Fh=f.x.curr()*cos(a3)+f.y.curr()*sin(a3);
            const qreal Fv=f.x.curr()*cos(a3+M_PI/2.0)+f.y.curr()*sin(a3+M_PI/2.0);

            if (Fv>0.0)
            {
                //qDebug() << "goto free2  F= " << Fv << "line=" << index1.curr() << "point=" << index2.curr();
                state.curr()=Free;
                return;
            }
            else if (state.prev()==Line1_to_Line2_fixed)
            {
//                && right_side(object1,object2,index1.prev(),index2.prev()))

                //qDebug() << "state is Point1_to_Line2_fixed prev state was line1_line2";
            }
        }

    }
    else if (state.curr()==Line1_to_Line2_fixed)
    {
        //return;
        const ShapePolygon & p1=dynamic_cast<const ShapePolygon&> (*object1->shape);
        const ShapePolygon & p2=dynamic_cast<const ShapePolygon&> (*object2->shape);
        const int i1=index1.curr();
        const int i2=index2.curr();
        const int i1p1=(i1+1)%p1.points.size();
        const int i2p1=(i2+1)%p2.points.size();
//        const int i1m1=(i1+p1.points.size()-1)%p1.points.size();
//        const int i2m1=(i2+p2.points.size()-1)%p2.points.size();

        const qreal & theta1=p1.angle.at(i1);
        const qreal alpha2=theta1+M_PI-p1.inner_angle.at(i1);

        const qreal a3=alpha2+object1->theta.curr();

        const qreal Fh=f.x.curr()*cos(a3)+f.y.curr()*sin(a3);
        const qreal Fv=f.x.curr()*cos(a3+M_PI/2.0)+f.y.curr()*sin(a3+M_PI/2.0);

        QPointF o2c=length.curr()*QPointF(cos(angle.curr()),sin(angle.curr()));

        const QPointF o1p1=p1.radius.at(i1  )*QPointF(cos(p1.angle.at(i1  )),sin(p1.angle.at(i1  )));
        const QPointF o1p2=p1.radius.at(i1p1)*QPointF(cos(p1.angle.at(i1p1)),sin(p1.angle.at(i1p1)));
        const QPointF o2p1=o2c+p2.radius.at(i2  )*QPointF(cos(angle2.curr()+p2.angle.at(i2  )),sin(angle2.curr()+p2.angle.at(i2  )));
        const QPointF o2p2=o2c+p2.radius.at(i2p1)*QPointF(cos(angle2.curr()+p2.angle.at(i2p1)),sin(angle2.curr()+p2.angle.at(i2p1)));

        QPointF v=o1p2-o1p1;
        v/=vector_length(v);

        const qreal o1p1x=o1p1.x()*v.x()+o1p1.y()*v.y();
        const qreal o1p2x=o1p2.x()*v.x()+o1p2.y()*v.y();
        const qreal o2p1x=o2p1.x()*v.x()+o2p1.y()*v.y();
        const qreal o2p2x=o2p2.x()*v.x()+o2p2.y()*v.y();
        const qreal y=-(o1p1.y()*v.x()-o1p1.x()*v.y());

        qreal x1,x2;
        bool a1,a2;
        if (o1p1x>o2p2x)
            x1=o1p1x,a1=true;
        else
            x1=o2p2x,a1=false;

        if (o1p2x<o2p1x)
            x2=o1p2x,a2=true;
        else
            x2=o2p1x,a2=false;

        const qreal F1=-(Fh*y - tau1.curr() + Fv*x2)/(x1 - x2);
        const qreal F2= (Fh*y - tau1.curr() + Fv*x1)/(x1 - x2);

 //       qDebug() << "Fh" << Fh << "Fv" << Fv;
  //      qDebug() << "F1" << F1 << "F2" << F2;

        const qreal negF=-0.0;

        if (F1<negF && F2<negF)
        {
            //qDebug() << "F1<0 && F2<0";
            //state.curr()=Free;
            //post_iteration();
            //return;
        }

        if (a2 && F1<negF)
        {

            state.curr()=Point1_to_Line2_fixed;
         //   qDebug()<< "lossnar a2 && F1<0.0";

            index2.curr()=i1p1;
            index1.curr()=i2;

            QPointF p=o1p2-o2c;
            length.curr()=vector_length(p);
            angle.curr()=atan2(p.y(),p.x())-angle2.curr();
            return;
        }

        if (!a2 && F1<negF)
        {
            state.curr()=Line1_to_Point2_fixed;
          //  qDebug()<< "lossnar !a2 && F1<0.0";

            index1.curr()=i1;
            index2.curr()=i2;

            QPointF p=o2p1;
            length.curr()=vector_length(p);
            angle.curr()=atan2(p.y(),p.x());
            return;
        }

        if (!a1 && F2<negF)
        {
            state.curr()=Line1_to_Point2_fixed;
         //   qDebug()<< "lossnar !a1 && F2<0.0";

            index1.curr()=i1;
            index2.curr()=i2p1;

            QPointF p=o2p2;
            length.curr()=vector_length(p);
            angle.curr()=atan2(p.y(),p.x());
            return;
        }

         if (a1 && F2<negF)
         {
        //     qDebug()<< "lossnar a1 && F2<0.0";
             state.curr()=Point1_to_Line2_fixed;

             index2.curr()=i1;
             index1.curr()=i2;

             QPointF p=o1p1-o2c;
             length.curr()=vector_length(p);
             angle.curr()=atan2(p.y(),p.x())-angle2.curr();
             return;
         }
    }
}

#include <QTransform>

QLineF Friction::get_contact_line(bool world_coordinates, int history)
{
    const ShapePolygon  & p1=dynamic_cast<const ShapePolygon&> (*object1->shape);
    const ShapePolygon  & p2=dynamic_cast<const ShapePolygon&> (*object2->shape);
    const int & i1=index1.at(history);
    const int & i2=index2.at(history);
    int i1p1=(i1+1)%p1.points.size();
    int i2p1=(i2+1)%p2.points.size();
    QLineF l;

    QPointF offset=length.at(history)*QPointF(cos(angle.at(history)),sin(angle.at(history)));
    QPointF point1,point2;

    point1=p1.radius.at(i1)*QPointF(cos(p1.angle.at(i1)),sin(p1.angle.at(i1)));
    point2=offset+p2.radius.at(i2p1)*QPointF(cos(angle2.at(history)+p2.angle.at(i2p1)),sin(angle2.at(history)+p2.angle.at(i2p1)));
    if (sin(atan2(point2.y(),point2.x())-p1.angle.at(i1))>0.0) l.setP1(point2); else l.setP1(point1);

    point1=p1.radius.at(i1p1)*QPointF(cos(p1.angle.at(i1p1)),sin(p1.angle.at(i1p1)));
    point2=offset+p2.radius.at(i2)*QPointF(cos(angle2.at(history)+p2.angle.at(i2)),sin(angle2.at(history)+p2.angle.at(i2)));
    if (sin(atan2(point2.y(),point2.x())-p1.angle.at(i1p1))<0.0) l.setP2(point2); else l.setP2(point1);

    if (world_coordinates)
    {
        QTransform a;
        a.translate(object1->p.x.at(history),object1->p.y.at(history));
        a.rotateRadians(object1->theta.at(history));
        l=a.map(l);
    }
    return l;

}

void Friction::update_graphics(int history)
{
    if (state.at(history)==Line1_to_Point2_fixed || state.at(history)==Point1_to_Line2_fixed)
    {
        item->setVisible(true);
        item2->setVisible(false);
        Mechanic2DObject *o1;
        if (state.at(history)==Line1_to_Point2_fixed)
            o1=object1;
        else
            o1=object2;

        item->setPos(o1->p.x.at(history)+length.at(history)*cos(angle.at(history)+o1->theta.at(history)),o1->p.y.at(history)+length.at(history)*sin(angle.at(history)+o1->theta.at(history)));
    }
    else if (state.at(history)==Line1_to_Line2_fixed)
    {
        item->setVisible(true);
        item2->setVisible(true);
        QLineF l=get_contact_line(true, history);
        item->setPos(l.p1());
        item2->setPos(l.p2());

    }
    else
    {
        item->setVisible(false);
        item2->setVisible(false);
    }

 //   item->setPos(object1->p.x.curr()+length1.curr.toDouble()*cos(angle1.curr.toDouble()+object1->theta.curr()),object1->p.y.curr()+length1.curr.toDouble()*sin(angle1.curr.toDouble()+object1->theta.curr()));
}
