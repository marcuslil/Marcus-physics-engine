#ifndef ENGINESETTINGS_H
#define ENGINESETTINGS_H

#include <QDialog>
#include "physicsengine.h"

namespace Ui {
class EngineSettings;
}

class EngineSettings : public QDialog
{
    Q_OBJECT
    
public:
    explicit EngineSettings(QWidget *parent = 0);
    ~EngineSettings();
    int update_graphics_time;
    PhysicsEngine *engine;
    void data_to_gui();

private:
    Ui::EngineSettings *ui;

private slots:
    void gui_to_data();
};

#endif // ENGINESETTINGS_H
