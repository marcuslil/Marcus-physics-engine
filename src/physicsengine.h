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
class Variable;
template <class T> class VariableT;
class Energy;

typedef VariableT<qreal> RealVariable;

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
    bool iteration();
    void init();
    void clear();
    void enableHistory(bool enable=true);
    inline int historySize() {return history_size;};
    qreal acc_energy_error;
    qreal energy_error_check;
//private:
    QList<PhysicsObject*> objects;
    QList<RealVariable*> eq_variables;
    QList<Energy*> energies;
    QList<Variable*> variables;
    QVector<char> curr,prev,prev_t,history;
    friend class PhysicsObject;
    friend class Variable;
    bool history_enabled;
    int history_size;
    void register_variables(PhysicsObject* object, bool eq_variable,Variable* v1,Variable* v2=0,Variable* v3=0,Variable* v4=0,Variable* v5=0,Variable* v6=0,Variable* v7=0,Variable* v8=0,Variable* v9=0,Variable* v10=0);
    void register_energies(PhysicsObject* object, Energy* e1,Energy* e2=0,Energy* e3=0,Energy* e4=0,Energy* e5=0,Energy* e6=0,Energy* e7=0,Energy* e8=0,Energy* e9=0,Energy* e10=0);
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
    void register_eq_variables(RealVariable* v1,RealVariable* v2=0,RealVariable* v3=0,RealVariable* v4=0,RealVariable* v5=0,RealVariable* v6=0,RealVariable* v7=0,RealVariable* v8=0,RealVariable* v9=0,RealVariable* v10=0);
    void register_energies(Energy* e1,Energy* e2=0,Energy* e3=0,Energy* e4=0,Energy* e5=0,Energy* e6=0,Energy* e7=0,Energy* e8=0,Energy* e9=0,Energy* e10=0);
};

class Variable
{
public:
    Variable(const QString & name);
    virtual QString curr_to_string() =0;
    virtual int size() =0;
    virtual void initialize() =0;
    inline const QString & name() {return _name;}
    PhysicsObject *object;
    int nr;
protected:
    int pos;
    QString _name;
    PhysicsEngine *engine;
    friend class PhysicsEngine;
};

template <class T>
class VariableT : public Variable
{
public:
    VariableT(const QString & name)
        : Variable(name)
    {
        init=0;
    }
    inline virtual void initialize()
    {
        curr()=init;
    }
    inline T & at(int history)
    {
        if (history<0 || history>=engine->history_size) return curr();
        else return *reinterpret_cast<T*>(engine->history.data()  +pos + history*engine->curr.size());
    }
    inline T & curr()   {return *reinterpret_cast<T*>(engine->curr.data()  +pos);}
    inline T & prev()   {return *reinterpret_cast<T*>(engine->prev.data()  +pos);}
    inline T & prev_t() {return *reinterpret_cast<T*>(engine->prev_t.data()+pos);}
    inline QString curr_to_string() {return QString("%1").arg(curr());}
    T init;
    inline virtual int size() { return sizeof(T);}
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

#endif // PHYSICENGINE_H
