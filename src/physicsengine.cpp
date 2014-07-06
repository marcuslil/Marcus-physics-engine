#include "physicsengine.h"

#include <QDebug>
#include <math.h>

#ifdef USE_LAPACKE
#include <lapacke.h>
#endif

PhysicsEngine::PhysicsEngine()
 : t(parameters.t),k(parameters.k)
{
    settings.max_delta_t = 0.01;
    settings.min_delta_t = 0.001;
    settings.energy_error_check = 1e-3;
    settings.max_subitarations = 2;
    settings.e_check = false;
    settings.set_k = 1.0;
    settings.max_k = 0.9;
    settings.min_k = 0.7;
    settings.k_energy_conserv = false;
    settings.add_k = 0.001;
    parameters.t = 0.0;
    parameters.k = settings.set_k;
    parameters.acc_energy_error = 0.0;
    sub_iteration = 0;
    small_time_step = false;
    clear();
}

PhysicsEngine::~PhysicsEngine()
{
    clear();
}

void PhysicsEngine::init()
{
    parameters.t = 0.0;
    parameters.k = settings.set_k;
    parameters.acc_energy_error = 0.0;
    parameters.energy_error = 0.0;
    parameters_last_t = parameters;
    sub_iteration = 0;
    delta_t = settings.max_delta_t;
    small_time_step = false;

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
    prev = curr;
    prev_t = curr;
    hist_block_size = curr.size() + sizeof(parameters);
    history.clear();
    history_size = 0;
    add_history();

#if defined USE_ARMADILLO
    A.resize(eq_variables.size(),eq_variables.size());
    A.fill(0);
    B.resize(eq_variables.size());
    B.fill(0);
#elif defined USE_LAPACKE
    size = eq_variables.size();
    _A.resize(size*size);
    _A.fill(0);
    _B.resize(size);
    _B.fill(0);
    ipiv.resize(size + 2);
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
    energies.clear();
    curr.clear();
    history.clear();
    history_size = 0;
    sub_iteration = 0;
    init();
}

void PhysicsEngine::add_history()
{
    const int alloc_ahead = 10000;
    int alloc = ((history_size + alloc_ahead - 1) / alloc_ahead) * alloc_ahead + 1;
    history.resize(alloc * hist_block_size);
    memcpy(history.data() + history_size * hist_block_size, curr.data(), curr.size());
    memcpy(history.data() + history_size * hist_block_size + curr.size(), &parameters, sizeof(parameters));
    history_size++;
}

void PhysicsEngine::enableHistory(bool enable)
{
    if (!enable)
    {
        history.clear();
        history_size = 0;
        add_history();
    }
    history_enabled = enable;
}

void PhysicsEngine::resetHistory(int position)
{
    Q_ASSERT(position < history_size);
    history.resize((position + 1) * hist_block_size);
    history_size = position + 1;
    memcpy(curr.data(), history.data() + position * hist_block_size, curr.size());
    memcpy(&parameters, history.data() + position * hist_block_size + curr.size(), sizeof(parameters));
    prev = curr;
    prev_t = curr;
    sub_iteration = 0;
    delta_t = settings.max_delta_t;
    small_time_step = false;
}

const PhysicsEngine::Parameters & PhysicsEngine::parameters_at(int position) const
{
    if (position < 0) return parameters;
    return *(reinterpret_cast<const Parameters*>(history.data() + position * hist_block_size + curr.size()));
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
    if (!small_time_step)
        delta_t = settings.max_delta_t;

    if (delta_t < settings.min_delta_t)
    {
        delta_t = settings.max_delta_t;
        return false;
    }

 //   for(int i=0;i!=objects.size();i++)
 //       objects[i]->post_iteration();

#if defined USE_ARMADILLO
    A.fill(0);
    B.fill(0);
#elif defined USE_LAPACKE
    _A.fill(0);
    _B.fill(0);
#else
    matrix.fill(0);
#endif
    if (!settings.k_energy_conserv)
        parameters.k=settings.set_k;

    for(int i=0;i!=objects.size();i++)
        objects[i]->setup_equations();

#if defined USE_ARMADILLO
    bool solveok = true;
    arma::vec X;
    try
    {
        X=arma::solve(A,B);
    }
    catch(std::runtime_error /*e*/)
    {
        solveok = false;
    }
    if (solveok)
        for (int v=0;v!=eq_variables.size();v++)
            eq_variables[v]->curr()=X(v);
#elif defined USE_LAPACKE
    int nrhs = 1, info = 0;
    LAPACK_dgesv(&size, &nrhs, _A.data(), &size, ipiv.data(), _B.data(), &size, &info);
    //    int info = LAPACKE_dgesv( LAPACK_COL_MAJOR, size, 1, _A.data(), size, ipiv.data(), _B.data(), size );
    bool solveok = (info == 0);
    if (solveok)
        memcpy(curr.data(), _B.data(), sizeof(qreal)*size);
#else
    QVector<qreal> A=matrix;
    bool solveok = solve(A,size);
    if (solveok)
        for (int v=0;v!=eq_variables.size();v++)
            eq_variables[v]->curr()=A.at(v*size2+size);
#endif
    if (!solveok)
    {
        delta_t /= 1.5;
        if (delta_t < settings.min_delta_t)
        {
            delta_t = settings.max_delta_t;
            small_time_step = false;
            parameters = parameters_last_t;
            qDebug() << "unsolveable matrix";
            return false;
        }
        small_time_step = true;
        return true;
    }

    for(int i=0;i!=objects.size();i++)
        objects[i]->post_iteration();

    if (sub_iteration < settings.max_subitarations)
    {
        sub_iteration++;
        prev=curr;
        return true;
    }

    sub_iteration=0;

    energy_matix.fill(false);
    for (int o=0;o!=objects.size();o++)
        objects.at(o)->calc_energy_diff();

    parameters.energy_error = 0.0;
    for (int e=0;e!=energies.size();e++)
    {
 //       qDebug() << energies.at(e)->object->name << " " << energies.at(e)->name << " " <<energies.at(e)->delta;
        parameters.energy_error += energies.at(e)->delta;
       // if (fabs(energies.at(e)->delta)> 0.1)
       //     qDebug() << energies.at(e)->name << energies.at(e)->delta;
    }
       // qDebug() << "Enery delta " << Edelta_error << acc_energy_error+Edelta_error << "dt" << delta_t;

    parameters.acc_energy_error += parameters.energy_error;
    parameters.energy_error /= delta_t;
    if (settings.e_check && fabs(parameters.energy_error) > settings.energy_error_check)
    {
        delta_t /= 2.0;
        curr = prev_t;
        prev = prev_t;

        if (delta_t < settings.min_delta_t)
        {
            qDebug() << fabs(parameters.energy_error);
            delta_t = settings.max_delta_t;
            small_time_step = false;
            parameters = parameters_last_t;
            return false;
        }
        small_time_step = true;
        return true;
    }

    prev_t=curr;
    prev=curr;
    parameters_last_t = parameters;

    if (settings.k_energy_conserv)
    {
        if (parameters.acc_energy_error<0) parameters.k=parameters.k-settings.add_k;
        else parameters.k=parameters.k+settings.add_k;
        if (parameters.k>settings.max_k) parameters.k=settings.max_k;
        if (parameters.k<settings.min_k) parameters.k=settings.min_k;
    }

    parameters.t += delta_t;
    small_time_step = false;

    if (history_enabled) add_history();
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

#ifndef USE_LAPACKE
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
#endif
