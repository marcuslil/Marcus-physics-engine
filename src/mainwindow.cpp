#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include "mechanics2d.h"
#include "pendlum.h"
#include <QElapsedTimer>
#include <QGraphicsItem>
#include "math.h"
#include <QWheelEvent>
#include "pendlumdialog.h"
#include "connections.h"
#include "friction.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(&timer,SIGNAL(timeout()),this,SLOT(time_step()));
    timer.setSingleShot(false);
    ui->graphicsView->scale(5,5);
    running=false;
    enginesettings.engine=&engine;
    engine.enableHistory();

    ui->graphicsView->installEventFilter(this);

    on_setup_activated(ui->setup->itemText(0));
}

MainWindow::~MainWindow()
{
    ui->graphicsView->setScene(0);
    delete ui;
}

void MainWindow::on_start_stop_clicked()
{
    if (running)
    {
        ui->step->setEnabled(true);
        ui->start_stop->setText("Start");
        timer.stop();
        ui->slide_history->setMaximum(engine.historySize());
        ui->spin_history->setMaximum(engine.historySize());
        ui->slide_history->setEnabled(true);
        ui->spin_history->setEnabled(true);
    }
    else
    {
        ui->step->setEnabled(false);
        ui->start_stop->setText("Stop");
        timer.start(enginesettings.update_graphics_time);
        ui->slide_history->setEnabled(false);
        ui->spin_history->setEnabled(false);
    }
    running=!running;
}

void MainWindow::update_graphics(qint64 sim_time)
{
    qint64 draw_time;
    QElapsedTimer a;
    a.start();
    world->update_graphics();
    draw_time=a.nsecsElapsed();
    used_cpu_time_drawing+=draw_time;
    bool tosloow=sim_time>0 && (draw_time+sim_time)/1e6>=enginesettings.update_graphics_time;
    ui->times->setText(QString("%1/%2/%3 %4").arg(engine.t).arg(used_cpu_time_simulation/1e9).arg(used_cpu_time_drawing/1e9).arg(tosloow?"Lagging":""));
    ui->lost->setText(QString("%1").arg(engine.acc_energy_error));
}

void MainWindow::time_step()
{
    qreal stop=engine.t+enginesettings.update_graphics_time/1000.0;
    qint64 sim_time;
    QElapsedTimer a;
    a.start();
    bool it_ok=false;
    while(engine.t<stop && a.nsecsElapsed()/1e6<=enginesettings.update_graphics_time && (it_ok=engine.iteration()));
    if (!it_ok) on_start_stop_clicked();
    sim_time=a.nsecsElapsed();
    used_cpu_time_simulation+=sim_time;
    update_graphics(sim_time);
}

void MainWindow::on_step_clicked()
{
    qint64 sim_time;
    QElapsedTimer a;
    a.start();
    engine.iteration();
    sim_time=a.elapsed();
    used_cpu_time_simulation+=sim_time;
    update_graphics(sim_time);
    ui->slide_history->setMaximum(engine.historySize());
    ui->spin_history->setMaximum(engine.historySize());
}

void MainWindow::on_variables_clicked()
{
    QString s=QString("t=%1\n").arg(engine.t);
    for (int v=0;v!=engine.variables.size();v++)
        s+=engine.variables.at(v)->object->name+"."+engine.variables.at(v)->name()+QString("=%1\n").arg(engine.variables.at(v)->curr_to_string());
    if (engine.k_energy_conserv) s+=QString("k=%1").arg(engine.k);
    ui->vars->setText(s);
}

void MainWindow::on_setup_activated(const QString &arg1)
{
    if (running) on_start_stop_clicked();

    if (arg1=="Pendlum test" && pendlumdialog.exec()!=QDialog::Accepted	) return;


    used_cpu_time_simulation=0;
    used_cpu_time_drawing=0;

    engine.clear();
    world=new Mechanic2DWorld(&engine,"world");
    ui->graphicsView->setScene(&world->scene);
    ui->graphicsView->setTransform(QTransform());
    ui->graphicsView->scale(5,5);
    engine.set_delta_t=0.01;
    if (arg1=="build 2 demo")
    {
        qreal ph=1;
        qreal l=10,g=9.82;

        MassObject *rect=new MassObject(world,"box1");
        rect->p.y.init=l*cos(ph);
        rect->p.x.init=l*sin(ph);
        rect->theta.init=M_PI_2-ph;

        DownForce *df=new DownForce(rect,"downforce box1");
        df->g=g;
        new CompoundPendlum(rect,"CompoundPendlum box1",0,0);

        MassObject *rect2=new MassObject(world,"box 2");
        rect2->p.x.init=-30;
        new DownForce(rect2,"downforce box2");
        new SimplePendlum(rect2,"simple pendlum box2",-40,0);

        MassObject *rect3=new MassObject(world,"box 3");
        new SpringWallForce(rect3,"Spring wall force box 3");
        DownForce *df3=new DownForce(rect3,"downforce box 3");
        df3->g=g;

        MassObject *rect4=new MassObject(world,"box 4");
        rect4->shape->item->setBrush(Qt::blue);
        rect4->p.x.init=10;
        rect4->m=1;
        rect4->v.y.init=10;
        LoseConnection *c=new LoseConnection(rect4,rect3,"connect 4 and 3");
        c->l=10;

        c=new LoseConnection(rect,rect3,"connect 0 and 3");
        c->l=10;
    }
    else if (arg1=="build 1 demo")
    {        
        qreal ph=1;
        qreal l=10,g=9.82;

        MassObject *rect=new MassObject(world,"box right");
        rect->p.y.init=l*cos(ph);
        rect->p.x.init=l*sin(ph);
        rect->theta.init=M_PI_2-ph;
        DownForce *df=new DownForce(rect,"downforce right");
        df->g=g;
        new CompoundPendlum(rect,"Compound pendlum",0,0);

        MassObject *rect2=new MassObject(world,"box left");
        rect2->p.x.init=-30;
        new DownForce(rect2,"downforce left");
        new SimplePendlum(rect2,"Simple pendlum",-40,0);

        MassObject *rect3=new MassObject(world,"free fall box");
        new SpringWallForce(rect3,"spring force");
        DownForce *df3=new DownForce(rect3,"downforce free fall");
        df3->g=g;
    }
    else if (arg1=="free fall")
    {
        MassObject *rect=new MassObject(world,"box 1");
        rect->p.y.init=-40;
        rect->m=1;
        new DownForce(rect,"down force");
        engine.set_delta_t=0.001;
    }
    else if (arg1=="bounce")
    {
        MassObject *rect3=new MassObject(world,"free fall box");
        new SpringWallForce(rect3,"spring force");
        new DownForce(rect3,"downforce free fall");
    }
    else if (arg1=="Pendlum test" )
    {
        MassObject *box=new MassObject(world,"box 1");
        qreal ph2=M_PI_2-pendlumdialog.theta*M_PI/180.0;
        box->p.x.init=pendlumdialog.l*cos(ph2);
        box->p.y.init=pendlumdialog.l*sin(ph2);
        box->v.x.init=-pendlumdialog.v*sin(ph2);
        box->v.y.init=pendlumdialog.v*cos(ph2);
        box->m=pendlumdialog.mass;
        box->I=pendlumdialog.I;

        DownForce *df=new DownForce(box,"downforce 1");
        df->g=pendlumdialog.g;

        if (pendlumdialog.type==0)
        {
            new SimplePendlum(box,"simple pendlum");
            if (pendlumdialog.math)
            {
                qreal T=2*M_PI*sqrt(pendlumdialog.l/pendlumdialog.g);
                qreal m=box->p.x.init;
                new MathPendlum(world,"math pendlum",0,pendlumdialog.l+3,T,m);
            }
        }
        else if (pendlumdialog.type==1)
        {
            box->theta.init=ph2;
            box->w.init=pendlumdialog.v/pendlumdialog.l;
            new CompoundPendlum(box,"compound pendlum",0,0);
            if (pendlumdialog.math)
            {
                qreal I2=pendlumdialog.I+pendlumdialog.mass*pendlumdialog.l*pendlumdialog.l;
                qreal T=2*M_PI*sqrt(I2/pendlumdialog.g/pendlumdialog.mass/pendlumdialog.l);
                qreal m=box->p.x.init;
                new MathPendlum(world,"math pendlum",0,pendlumdialog.l+3,T,m);
            }
        }
        else if (pendlumdialog.type==2)
        {
           // box->theta.init=ph2;
           // box->w.init=pendlumdialog.v/pendlumdialog.l;
            new StringPendlum(box,"compound pendlum",0,0);
        }

        engine.set_delta_t=0.0001;
    }
    else if (arg1=="collision")
    {
        MassObject *box=new MassObject(world,"box 1");
        box->p.y.init=5;
        box->p.x.init=-2;
        box->v.y.init=-1;
        box->theta.init=-2;
//        box->w.init=-0.3;
        QVector<QPointF> p;
        p.append(QPointF(5,-3));
        p.append(QPointF(3,4));
        p.append(QPointF(0,2));
        MassObject *box2=new MassObject(world,"box 2",new ShapePolygon(p));
        box2->w.init=2;
        box2->I=20;
        new Friction(box,box2,"friction 1");
//        new Friction(box2,box,"friction 1");
    }
    else if (arg1=="collision2")
    {
        MassObject *box=new MassObject(world,"box 1");
        box->p.y.init=5;
        box->v.y.init=-1;
        box->w.init=-0.3;
        QVector<QPointF> p;
        p.append(QPointF(5,-3));
        p.append(QPointF(3,4));
        p.append(QPointF(0,2));
        MassObject *box2=new MassObject(world,"box 2",new ShapePolygon(p));
        box2->w.init=0.8;
        box2->I=20;
        new Friction(box,box2,"friction 1");

    }
    else if  (arg1=="collision3")
    {
        QVector<QPointF> p1;
        p1.append(QPointF(-1,-1.0));
        p1.append(QPointF(1.1,-1.2));
        p1.append(QPointF(1,0.8));
        p1.append(QPointF(-1,1.3));
        MassObject *box=new MassObject(world,"box 1",new ShapePolygon(p1));
        box->p.y.init=-1.5;
        box->v.y.init=3;
        box->theta.init=M_PI/4+0.2;
        QVector<QPointF> p2;
        p2.append(QPointF(-1,-1.0));
        p2.append(QPointF(1,-0.9));
        p2.append(QPointF(1,1));
        p2.append(QPointF(-1,1));
        MassObject *box2=new MassObject(world,"box 2",new ShapePolygon(p2));
        box2->m=1000;
        box2->I=10;
        box2->theta.init=-0.4; //31;
        box2->p.y.init=1.5;
        box2->w.init=0.1;
        box2->shape->item->setBrush(Qt::red);

        new Friction(box2,box,"friction 1");
        ui->graphicsView->scale(10,10);

    }
    else if (arg1=="collision4")
    {
        QVector<QPointF> p1;
        p1.append(QPointF(-2,-2.0));
        p1.append(QPointF(2,-2));
        p1.append(QPointF(2,2));
        p1.append(QPointF(-2,2));
        MassObject *box=new MassObject(world,"box 1",new ShapePolygon(p1));
        box->p.y.init=1.5;
        box->m=100;
        box->I=100;
      //  box->w.init=0.1;
        MassObject *box2=new MassObject(world,"box 2");
        box2->p.y.init=-1.6;
        box2->p.x.init=1.9;
        box2->theta.init=-0.00;
        new DownForce(box2,"downforce");
        new Friction(box,box2,"friction 1");
        //new Fix(box,"fix",true);
       // new ConstTorque(box2,"const torque",-2.0);
        ui->graphicsView->scale(5,5);
    }
    else if (arg1=="collision5")
    {
        QVector<QPointF> p1;
        p1.append(QPointF(-1,-2.0));
        p1.append(QPointF(1,-2));
        p1.append(QPointF(1,2));
        p1.append(QPointF(-1,2));
        MassObject *box=new MassObject(world,"box 1",new ShapePolygon(p1));
        box->p.y.init=1.5;
        box->theta.init=0.0; //-0.6;
        box->m=10000;
        box->I=10000;
      //  box->w.init=0.1;
        MassObject *box2=new MassObject(world,"box 2");
        box2->p.y.init=6;
        box2->p.x.init=0;
        box2->theta.init=-0.25+0*M_PI/1.0;
        box2->I=1;
        new DownForce(box,"downforce");
        new Friction(box2,box,"friction 1");
        new Fix(box2,"fix",true);
       // new ConstTorque(box2,"const torque",-8.0);
        ui->graphicsView->scale(5,5);
    }
    else if (arg1=="collision6")
    {
        QVector<QPointF> p1;
        p1.append(QPointF(-10,-2.0));
        p1.append(QPointF(10,-2));
        p1.append(QPointF(10,2));
        p1.append(QPointF(-10,2));
        MassObject *box=new MassObject(world,"box 1",new ShapePolygon(p1));
        box->p.y.init=1.5;
        box->theta.init=-0.6;
        box->m=10000;
        box->I=10000;
      //  box->w.init=0.1;
        MassObject *box2=new MassObject(world,"box 2");
        box2->p.y.init=-5;
        box2->p.x.init=0;
        box2->theta.init=-0.25+0*M_PI/1.0;
        box2->I=0.5;
        new DownForce(box2,"downforce");
        new Friction(box,box2,"friction 1");
        new Fix(box,"fix",true);
       // new ConstTorque(box2,"const torque",-8.0);
        ui->graphicsView->scale(5,5);
    }
    else if (arg1=="collision7")
    {
        QVector<QPointF> p1;
        p1.append(QPointF(-10,-2.0));
        p1.append(QPointF(10,-2));
        p1.append(QPointF(10,2));
        p1.append(QPointF(-10,2));
        MassObject *box=new MassObject(world,"box 1",new ShapePolygon(p1));
        box->p.y.init=1.5;
        box->theta.init=-0.6;
        box->m=10000;
        box->I=10000;
        MassObject *box3=new MassObject(world,"box 3",new ShapePolygon(p1));
        box3->p.y.init=4.5;
        box3->theta.init=-0.6+M_PI/2.0;
        box3->m=10000;
        box3->I=10000;
      //  box->w.init=0.1;
        MassObject *box2=new MassObject(world,"box 2");
        box2->p.y.init=-5;
        box2->p.x.init=0;
        box2->theta.init=-0.25+0*M_PI/1.0;
        box2->I=0.5;
        new DownForce(box2,"downforce");
        new Friction(box,box2,"friction 1");
        new Friction(box3,box2,"friction 1");
        new Fix(box,"fix",true);
       // new ConstTorque(box2,"const torque",-8.0);
        ui->graphicsView->scale(5,5);
    }
    else if(arg1=="tower")
    {
        QVector<QPointF> p1;
        p1.append(QPointF(-30,-2.0));
        p1.append(QPointF(30,-2));
        p1.append(QPointF(30,2));
        p1.append(QPointF(-30,2));
        MassObject *box1=new MassObject(world,"fbox 1",new ShapePolygon(p1));
        box1->p.x.init=-30;
        box1->theta.init=0.6;
        new Fix(box1,"");
        MassObject *box2=new MassObject(world,"fbox 2",new ShapePolygon(p1));
        box2->p.x.init=30;
        box2->theta.init=-0.6;
        new Fix(box2,"");
        p1.clear();
        p1.append(QPointF(-6,-2.0));
        p1.append(QPointF(6,-2));
        p1.append(QPointF(6,2));
        p1.append(QPointF(-6,2));
        MassObject *box3=new MassObject(world,"fbox 3",new ShapePolygon(p1));
        box3->p.x.init=0;
        box3->p.y.init=15;
        box3->theta.init=0;
        new Fix(box3,"");
        srand(0);

        MassObject *boxn[50];
        for (int i=0;i!=10;i++)
        {
            QVector<QPointF> p1;
            p1.append(QPointF(-1.5-0.5*rand()/RAND_MAX,-1.5-0.5*rand()/RAND_MAX));
            p1.append(QPointF(2,-1.5)); rand(); //-+-5*rand()/RAND_MAX));
            p1.append(QPointF(2,2));
            p1.append(QPointF(-1.5-0.5*rand()/RAND_MAX,2));
            boxn[i]=new MassObject(world,QString("box %1").arg(i),new ShapePolygon(p1));
            boxn[i]->p.x.init=2.0*rand()/RAND_MAX;
            boxn[i]->p.y.init=-i*10.0-10.0;
            new DownForce(boxn[i],"downforce");
            new Friction(boxn[i],box1,"");
            new Friction(boxn[i],box2,"");
            new Friction(boxn[i],box3,"");
            for (int i2=0;i2<i;i2++)
                new Friction(boxn[i],boxn[i2],"");

        }
    }
    else if(arg1=="collision8")
    {
        QVector<QPointF> p1;
        p1.clear();
        p1.append(QPointF(-6,-2.0));
        p1.append(QPointF(6,-2));
        p1.append(QPointF(6,2));
        p1.append(QPointF(-6,2));
        MassObject *box3=new MassObject(world,"fbox 3",new ShapePolygon(p1));
        box3->p.x.init=0;
        box3->p.y.init=15;
        box3->theta.init=0;
        new Fix(box3,"");

        p1.clear();
        p1.append(QPointF(-2,-2));
        p1.append(QPointF(2,-2));
        p1.append(QPointF(2,2));
        p1.append(QPointF(-2,2));
        MassObject *boxn = new MassObject(world,QString("box 0"),new ShapePolygon(p1));
        boxn->p.x.init=2.0;
        boxn->p.y.init=-10.0;
        boxn->theta.init = 0.1;
        new DownForce(boxn,"downforce");
        new Friction(boxn,box3,"");


    }
    else if (arg1=="t1")
    {
        MassObject *box=new MassObject(world,"box 1");
        box->p.x.init=-10;
        MassObject *box2=new MassObject(world,"box 2");
        box2->p.x.init=-5;
        new LoseConnection(box,box2,"c1");
        new DownForce(box2,"df");
        box=new MassObject(world,"box 3");
        box->p.x.init=5;
        box->I=0.001;
        box->theta.init=2;
        box2=new MassObject(world,"box 4");
        box2->p.x.init=10;
        new FixedConnection(box,box2,"c2",false);
        new DownForce(box2,"df2");
    }
    else if (arg1=="t2")
    {
        MassObject *box=new MassObject(world,"box 1");
        box->p.x.init=-10;
        box->I=1;
        MassObject *box2=new MassObject(world,"box 2");
        box2->p.x.init=-5;
        new FixedConnection(box,box2,"c2");
        new DownForce(box2,"df2");
        MassObject *box3=new MassObject(world,"box 2");
        box3->p.x.init=-25;
        new FixedConnection(box,box3,"c2");
    }
    engine.init();
    enginesettings.data_to_gui();
    update_graphics();
}

void MainWindow::on_sim_settings_clicked()
{
    enginesettings.show();
}

bool MainWindow::eventFilter(QObject *object, QEvent *event)
{
    if (object!=ui->graphicsView) return QMainWindow::eventFilter(object,event);
    if (event->type() != QEvent::Wheel) return false;
    // Typical Calculations (Ref Qt Doc)
    QWheelEvent *event2=dynamic_cast<QWheelEvent*>(event);
        const int degrees = event2->delta() / 8;
        int steps = degrees / 15;

        QTransform m=ui->graphicsView->transform();
        if (steps>0)
            m.scale(1.1,1.1);
        else
            m.scale(1.0/1.1,1.0/1.1);
        ui->graphicsView->setTransform(m);
        return true;
        /*

        // Declare below as class member vars and set default values as below
        // qreal h11 = 1.0
        // qreal h12 = 0
        // qreal h21 = 1.0
        // qreal h22 = 0

        double scaleFactor = 1.0; //How fast we zoom
        const qreal minFactor = 1.0;
        const qreal maxFactor = 10.0;
        if(steps > 0)
        {
            qreal h11 = (h11 >= maxFactor) ? h11 : (h11 + scaleFactor);
            qreal h22 = (h22 >= maxFactor) ? h22 : (h22 + scaleFactor);
        }
        else
        {
            qreal h11 = (h11 <= minFactor) ? minFactor : (h11 - scaleFactor);
            qreal h22 = (h22 <= minFactor) ? minFactor : (h22 - scaleFactor);
        }

        ui->graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        ui->graphicsView->setTransform(QTransform(h11, h12, h21, h22, 0, 0));*/
}

void MainWindow::closeEvent(QCloseEvent *)
{
    enginesettings.close();
}

void MainWindow::on_check_history_clicked(bool checked)
{
    engine.enableHistory(checked);
    ui->slide_history->setMaximum(engine.historySize());
    ui->spin_history->setMaximum(engine.historySize());
}

void MainWindow::on_slide_history_sliderMoved(int position)
{
    ui->spin_history->setValue(position);
    world->update_graphics(position);
}

void MainWindow::on_spin_history_valueChanged(int arg1)
{
    ui->slide_history->setValue(arg1);
    world->update_graphics(arg1);
}
