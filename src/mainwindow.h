#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "physicsengine.h"
#include "mechanics2d.h"
#include "enginesettings.h"
#include "pendlumdialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    PhysicsEngine engine;
    Mechanic2DWorld *world;
    QTimer timer;
    bool running;
    bool eventFilter(QObject *, QEvent *);
    void closeEvent(QCloseEvent *);

private slots:
    void update_graphics(qint64 sim_time=0);
    void on_start_stop_clicked();
    void time_step();
    void on_step_clicked();
    void on_variables_clicked();
    void on_setup_activated(const QString &arg1);
    void on_sim_settings_clicked();
    void on_check_history_clicked(bool checked);
    void on_slide_history_sliderMoved(int position);
    void on_spin_history_valueChanged(int arg1);

private:
    Ui::MainWindow *ui;
    qint64 used_cpu_time_simulation,used_cpu_time_drawing;
    EngineSettings enginesettings;
    PendlumDialog pendlumdialog;
};

#endif // MAINWINDOW_H
