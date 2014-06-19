#ifndef PHYSICENGINE_H
#define PHYSICENGINE_H

#include <QList>
#include <QString>
#include <QVector>
#include <QVariant>
#include "math.h"

//#define  USE_ARMADILLO

#ifdef USE_ARMADILLO
#include <armadillo>
#endif

class PhysicsObject;

class Variable
{
public:
    Variable(const QString & name);
    qreal operator=(const qreal & value);
    qreal curr,last_t;
    bool history;
    QList<qreal> hist;
    int nr;
    QString name;
    PhysicsObject *object;
};

class Energy
{
public:
    Energy(const QString & name);
    int nr;
    qreal delta;
    PhysicsObject *object;
    QString name;
};

class State
{
public:
    State(const QString & name,const QVariant & init);
    int nr;
    QString name;
    QVariant curr,last_t;
    const State & operator=(const QVariant & value);
    PhysicsObject *object;
};

class PhysicsEngine
{
public:
    PhysicsEngine();
    ~PhysicsEngine();
    qreal set_delta_t,delta_t,t,set_k,k,max_k,min_k,add_k;
    bool k_energy_conserv;
    int sub_iteration,max_subitarations,e_check_iteration,max_e_check_iterations;
#ifdef USE_ARMADILLO
//QVector<qreal> matrix;
    arma::mat A;
    arma::vec B;
#else
    QVector<qreal> matrix;
    int size,size2;
    inline qreal & A(int y,int x) {return matrix[y*size2+x];}
    inline qreal & B(int y) {return matrix[y*size2+size];}
    static bool solve(QVector<qreal> & A,int size);
    static void dumpA(const QVector<qreal> & A,int size);
#endif
    QVector<bool> energy_matix;
    inline bool & Em(int e1,int e2) {return energy_matix[e1*energies.size()+e2];}
    void iteration();
    void clear();
    qreal acc_energy_error;
    qreal energy_error_check;
//private:
    QList<PhysicsObject*> objects;
    QList<Variable*> variables;
    QList<Energy*> energies;
    QList<State*> states;
    friend class PhysicsObject;
    void register_variables(PhysicsObject* object,Variable* v1,Variable* v2=0,Variable* v3=0,Variable* v4=0,Variable* v5=0,Variable* v6=0,Variable* v7=0,Variable* v8=0,Variable* v9=0,Variable* v10=0);
    void register_energies(PhysicsObject* object, Energy* e1,Energy* e2=0,Energy* e3=0,Energy* e4=0,Energy* e5=0,Energy* e6=0,Energy* e7=0,Energy* e8=0,Energy* e9=0,Energy* e10=0);
    void register_states(PhysicsObject* object, State* s1,State* s2=0,State* s3=0,State* s4=0,State* s5=0,State* s6=0,State* s7=0,State* s8=0,State* s9=0,State* s10=0);
};

class PhysicsObject
{
public:
    PhysicsObject(PhysicsEngine *engine,const QString & name=QString());
    virtual ~PhysicsObject();

    PhysicsEngine *engine;
    virtual void calc_energy_diff() {}
    virtual void setup_equations()=0;
    virtual void post_iteration() {}
    QString name;
    inline qreal & A(int row,int col) {return engine->A(row,col);}
    inline qreal & B(int row) {return engine->B(row);}
protected:
    void register_variables(Variable* v1,Variable* v2=0,Variable* v3=0,Variable* v4=0,Variable* v5=0,Variable* v6=0,Variable* v7=0,Variable* v8=0,Variable* v9=0,Variable* v10=0);
    void register_energies(Energy* e1,Energy* e2=0,Energy* e3=0,Energy* e4=0,Energy* e5=0,Energy* e6=0,Energy* e7=0,Energy* e8=0,Energy* e9=0,Energy* e10=0);
    void register_states(State* s1,State* s2=0,State* s3=0,State* s4=0,State* s5=0,State* s6=0,State* s7=0,State* s8=0,State* s9=0,State* s10=0);
};

#endif // PHYSICENGINE_H
