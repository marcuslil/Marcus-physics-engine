#ifndef FRICTION_H
#define FRICTION_H

#include "mechanics2d.h"

class Friction : public PhysicsObject, public Graphics2D
{
public:
    Friction(Mechanic2DObject *object1,Mechanic2DObject *object2,const QString & name);
    Vector f;
    RealVariable tau1,tau2;
    VariableT<int> state,prevstate,index1,index2;
    RealVariable angle,angle2,length;
    Mechanic2DObject *object1,*object2;
    virtual void setup_equations();
    virtual void calc_energy_diff();
    virtual void post_iteration();
    virtual void update_graphics();
    enum {Free=0, Line1_to_Point2_fixed,Point1_to_Line2_fixed,Line1_to_Line2_fixed};
    QGraphicsEllipseItem *item,*item2;
private:
    QLineF get_contact_line(bool world_coordinates=false);

};

#endif // FRICTION_H
