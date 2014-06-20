#include "physicsengine.h"

#include <QDebug>

PhysicsEngine::PhysicsEngine()
{
    set_delta_t=0.1;
    t=0;
    energy_error_check=1e-3;
    sub_iteration=0;
    max_subitarations=2;
    e_check_iteration=0;
    max_e_check_iterations=0;
    set_k=1.0;
    max_k=0.9;
    min_k=0.7;
    k_energy_conserv=false;
    k=1;
    add_k=0.001;
    clear();
}

PhysicsEngine::~PhysicsEngine()
{
    clear();
}

void PhysicsEngine::init()
{
    t=0;
    acc_energy_error=0.0;
    for (int v=0;v!=eq_variables.size();v++)
    {
        eq_variables[v]->nr=v;
        eq_variables[v]->pos=curr.size();
        Q_ASSERT(eq_variables.at(v)->size()==sizeof(qreal));
        curr.resize(curr.size()+sizeof(qreal));
        eq_variables[v]->initialize();
    }
    for (int v=0;v!=variables.size();v++)
    {
        variables[v]->pos=curr.size();
        curr.resize(curr.size()+variables.at(v)->size());
        variables[v]->initialize();
    }
    prev=curr;
    prev_t=curr;

#ifdef USE_ARMADILLO
    A.resize(eq_variables.size(),eq_variables.size());
    A.fill(0);
    B.resize(eq_variables.size());
    B.fill(0);
#else
    size=eq_variables.size();
    size2=eq_variables.size()+1;
    matrix.resize(size*size2);
    matrix.fill(0);
#endif
}


void PhysicsEngine::clear()
{
    while(!objects.isEmpty())
        delete objects.last();
    eq_variables.clear();
    variables.clear();
    eq_variables.clear();
    energies.clear();
    curr.clear();
    init();
}

void PhysicsEngine::register_variables(PhysicsObject *object, bool eq_variable,Variable *v1, Variable *v2, Variable *v3, Variable *v4, Variable *v5, Variable *v6, Variable *v7, Variable *v8, Variable *v9, Variable *v10)
{
    Variable *vs[]={v1,v2,v3,v4,v5,v6,v7,v8,v9,v10};
    for (int v=0;vs[v];v++)
    {
        vs[v]->pos=-1;
        vs[v]->nr=-1;
        vs[v]->engine=this;
        vs[v]->object=object;
        if (eq_variable)
        {
            Q_ASSERT(dynamic_cast<RealVariable*>(vs[v])!=0);
            eq_variables.append(dynamic_cast<RealVariable*>(vs[v]));
        }
        else
            variables.append(vs[v]);
    }
}

void PhysicsEngine::register_energies(PhysicsObject *object,Energy *e1, Energy *e2, Energy *e3, Energy *e4, Energy *e5, Energy *e6, Energy *e7, Energy *e8, Energy *e9, Energy *e10)
{
    Energy *es[]={e1,e2,e3,e4,e5,e6,e7,e8,e9,e10};
    for (int e=0;es[e];e++)
    {
        es[e]->nr=energies.size();
        es[e]->object=object;
        energies.append(es[e]);
    }
    energy_matix.resize(energies.size()*energies.size());
}



bool PhysicsEngine::iteration()
{
    //qDebug() << "iteration";
    if (e_check_iteration==0)
        delta_t=set_delta_t;

 //   for(int i=0;i!=objects.size();i++)
 //       objects[i]->post_iteration();

#ifdef USE_ARMADILLO
    A.fill(0);
    B.fill(0);
#else
    matrix.fill(0);
#endif
    for(int i=0;i!=objects.size();i++)
        objects[i]->setup_equations();

#ifdef USE_ARMADILLO
    bool solveok=true;
    arma::vec X;
    try
    {
        X=arma::solve(A,B);
    }
    catch(std::runtime_error e)
    {
        solveok=false;
    }
    for (int v=0;solveok && v!=eq_variables.size();v++)
        if (abs(eq_variables[v]->curr()-X(v))>1e3) solveok=false;
    if (!solveok)
    {
        static int a=0;
//        qDebug() << "using pinv" << a++;
        X=arma::pinv(A,1e-5)*B;
    }
    for (int v=0;v!=eq_variables.size();v++)
        eq_variables[v]->curr()=X(v);

#else

    QVector<qreal> A=matrix;


    if (!solve(A,size))
    {
        qDebug() << "No solution" << e_check_iteration << delta_t;
        e_check_iteration++;
        delta_t/=2.1;
        return false;
//        exit(0);
    }


    for (int v=0;v!=eq_variables.size();v++)
        eq_variables[v]->curr()=A.at(v*size2+size);
#endif

    for(int i=0;i!=objects.size();i++)
        objects[i]->post_iteration();

    if (sub_iteration<max_subitarations)
    {
        sub_iteration++;
        prev=curr;
        return true;
    }

    sub_iteration=0;

    energy_matix.fill(false);
    for (int o=0;o!=objects.size();o++)
        objects.at(o)->calc_energy_diff();

    qreal Edelta_error=0;
    for (int e=0;e!=energies.size();e++)
    {
 //       qDebug() << energies.at(e)->object->name << " " << energies.at(e)->name << " " <<energies.at(e)->delta;
        Edelta_error+=energies.at(e)->delta;
    }
       // qDebug() << "Enery delta " << Edelta_error << acc_energy_error+Edelta_error << "dt" << delta_t;

    if (e_check_iteration<max_e_check_iterations && fabs(Edelta_error/delta_t*2.0)>energy_error_check)
    {
        delta_t=delta_t/2.0;
        curr=prev_t;
        prev=prev_t;

        e_check_iteration++;
        return true;
    }

    e_check_iteration=0;

  //  qDebug() << "pass edelta" << Edelta_error;

    prev_t=curr;
    prev=curr;

    acc_energy_error+=Edelta_error;
    if (k_energy_conserv)
    {
        if (acc_energy_error<0) k=k-add_k;
        else k=k+add_k;
        if (k>max_k) k=max_k;
        if (k<min_k) k=min_k;
    }
    else
        k=set_k;

    t=t+delta_t;
    return true;
}

Energy::Energy(const QString &name)
    : name(name)
{}

Variable::Variable(const QString &name)
    : _name(name)
{}


void PhysicsObject::register_eq_variables(RealVariable *v1, RealVariable *v2, RealVariable *v3, RealVariable *v4, RealVariable *v5, RealVariable *v6, RealVariable *v7, RealVariable *v8, RealVariable *v9, RealVariable *v10)
{
    engine->register_variables(this,true,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10);
}

void PhysicsObject::register_variables(Variable *v1, Variable *v2, Variable *v3, Variable *v4, Variable *v5, Variable *v6, Variable *v7, Variable *v8, Variable *v9, Variable *v10)
{
    engine->register_variables(this,false,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10);
}

void PhysicsObject::register_energies(Energy *e1, Energy *e2, Energy *e3, Energy *e4, Energy *e5, Energy *e6, Energy *e7, Energy *e8, Energy *e9, Energy *e10)
{
    engine->register_energies(this,e1,e2,e3,e4,e5,e6,e7,e8,e9,e10);
}



PhysicsObject::~PhysicsObject()
{
    engine->objects.removeOne(this);
}

#ifndef USE_ARMADILLO

bool PhysicsEngine::solve(QVector<qreal> &A,int size)
{

    int size2=size+1;
    A[0]=A.at(0);
    qreal *M=A.data(),tmp[size2];
#define Mat(y,x) M[y*size2+x]
    int y,x,y2;
    qreal a;

    int mcpylen2=size2*sizeof(qreal);

    for(int y=0;y!=size;y++)
    {

        if (Mat(y,y)==0)
        { // swap lines?
            if (y==size-1) return false;
            memcpy(tmp,&M[y*size2],mcpylen2);
            for (y2=y+1;y2<size;y2++)
            {
                if (Mat(y2,y)!=0)
                {
                   // qDebug() << "swapping line " << y << " with " << y2;
                    memcpy(&M[y*size2],&M[y2*size2],mcpylen2);
                    memcpy(&M[y2*size2],tmp,mcpylen2);
                    y2=size;
                }
            }
            if  (y2==size) return false;
        }

        // normalize line
        a=Mat(y,y);

        Mat(y,y)=1;
        for (x=y+1;x<size2;x++)
            Mat(y,x)/=a;
        for (int y2=y+1;y2<size;y2++)
        {
            a=Mat(y2,y);
            if (a != 0)
            { // add line to lines below
          //      qDebug() << "line " << y << " will be multiplied with " << a << "and addes to line " << y2;
                Mat(y2,y)=0;
                for (x=y+1;x<size2;x++)
                    Mat(y2,x)-=Mat(y,x)*a;
            }
        }

    }

    for(y=size-1;y>-1;y--)
    { // step 2

        for (int y2=y-1;y2>-1;y2--)
        {
            a=Mat(y2,y);
            if (a != 0)
            { // add line to lines above
        //        qDebug() << "line " << y << " will be multiplied with " << a << "and addes to line " << y2;
                Mat(y2,y)=0;
                for (x=y+1;x<size2;x++)
                    Mat(y2,x)-=Mat(y,x)*a;
            }
        }

    }
    return true;
}

void PhysicsEngine::dumpA(const QVector<qreal> &A, int size)
{
    for (int y=0;y!=size;y++)
    {
        QString s=QString("%1: ").arg(y);
        for (int x=0;x!=size;x++)
        {
            s=s+QString(" %1").arg(A.at(y*(size+1)+x));
        }
        s=s+QString(" = %1").arg(A.at(y*(size+1)+size));
        qDebug() << s;
    }
}

#endif
