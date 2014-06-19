#include "enginesettings.h"
#include "ui_enginesettings.h"
#include <QDebug>

EngineSettings::EngineSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::EngineSettings)
{
    ui->setupUi(this);
    update_graphics_time=0.01*1000.0;
}

EngineSettings::~EngineSettings()
{
    delete ui;
}

void EngineSettings::gui_to_data()
{
    qDebug() << Q_FUNC_INFO;
    bool ok;
    qreal val;
    int ival;

    ival=ui->max_iter_tstep->text().toInt(&ok);
    if (ok && ival>=0) engine->max_subitarations=ival;

    val=ui->energy->text().toDouble(&ok);
    if (ok && val>0.0) engine->energy_error_check=val;

    if (ui->energy_check->isChecked())
    {
        ival=ui->max_energy_iters->text().toInt(&ok);
        if (!ok || ival<=0)
            ival=1;
        engine->max_e_check_iterations=ival;
    }
    else
        engine->max_e_check_iterations=0;

     val=ui->grap_update->text().toDouble(&ok)*1000.0;
     if (val==0) val=1;
     if (ok && val>0) update_graphics_time=val;

     val=ui->set_delta_t->text().toDouble(&ok);
     if (ok && val>0)
         engine->set_delta_t=val;

     val=ui->set_k->text().toDouble(&ok);
     if (ok)
         engine->set_k=val;

    engine->k_energy_conserv=ui->update_k->isChecked();

    val=ui->add_k->text().toDouble(&ok);
    if (ok && val>0.0) engine->add_k=val;
    val=ui->min_k->text().toDouble(&ok);
    if (ok) engine->min_k=val;
    val=ui->max_k->text().toDouble(&ok);
    if (ok) engine->max_k=val;

    data_to_gui();
}

void EngineSettings::data_to_gui()
{

    qDebug() << Q_FUNC_INFO;
    ui->grap_update->setText(QString("%1").arg(update_graphics_time/1000.0));
    ui->set_delta_t->setText(QString("%1").arg(engine->set_delta_t));
    ui->max_iter_tstep->setText(QString("%1").arg(engine->max_subitarations));
    if (engine->max_e_check_iterations>0)
    {
        ui->energy_check->setChecked(true);
        ui->max_energy_iters->setEnabled(true);
        ui->max_energy_iters->setText(QString("%1").arg(engine->max_e_check_iterations));
        ui->energy->setEnabled(true);
    }
    else
    {
        ui->energy_check->setChecked(false);
        ui->max_energy_iters->setEnabled(false);
        ui->energy->setEnabled(false);
    }


    ui->set_k->setText(QString("%1").arg(engine->set_k));
    ui->update_k->setChecked(engine->k_energy_conserv);
    ui->add_k->setText(QString("%1").arg(engine->add_k));
    ui->min_k->setText(QString("%1").arg(engine->min_k));
    ui->max_k->setText(QString("%1").arg(engine->max_k));


    ui->set_k->setEnabled(!engine->k_energy_conserv);
    ui->add_k->setEnabled(engine->k_energy_conserv);
    ui->max_k->setEnabled(engine->k_energy_conserv);
    ui->min_k->setEnabled(engine->k_energy_conserv);

}
