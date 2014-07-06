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
    if (ok && ival>=0) engine->settings.max_subitarations=ival;

    val=ui->energy->text().toDouble(&ok);
    if (ok && val>0.0) engine->settings.energy_error_check=val;

    engine->settings.e_check = ui->energy_check->isChecked();

    val=ui->grap_update->text().toDouble(&ok)*1000.0;
    if (val==0) val=1;
    if (ok && val>0) update_graphics_time=val;

    val=ui->max_delta_t->text().toDouble(&ok);
    if (ok && val>0)
        engine->settings.max_delta_t=val;

    val=ui->min_delta_t->text().toDouble(&ok);
    if (ok && val>0)
        engine->settings.min_delta_t=val;

    val=ui->set_k->text().toDouble(&ok);
    if (ok)
        engine->settings.set_k=val;

    engine->settings.k_energy_conserv=ui->update_k->isChecked();

    val=ui->add_k->text().toDouble(&ok);
    if (ok && val>0.0) engine->settings.add_k=val;
    val=ui->min_k->text().toDouble(&ok);
    if (ok) engine->settings.min_k=val;
    val=ui->max_k->text().toDouble(&ok);
    if (ok) engine->settings.max_k=val;

    data_to_gui();
}

void EngineSettings::data_to_gui()
{
    qDebug() << Q_FUNC_INFO;
    ui->grap_update->setText(QString("%1").arg(update_graphics_time / 1000.0));
    ui->max_delta_t->setText(QString("%1").arg(engine->settings.max_delta_t));
    ui->min_delta_t->setText(QString("%1").arg(engine->settings.min_delta_t));
    ui->max_iter_tstep->setText(QString("%1").arg(engine->settings.max_subitarations));
    ui->energy->setText(QString("%1").arg(engine->settings.energy_error_check));
    ui->energy_check->setChecked(engine->settings.e_check);
    ui->energy->setEnabled(engine->settings.e_check);

    ui->set_k->setText(QString("%1").arg(engine->settings.set_k));
    ui->update_k->setChecked(engine->settings.k_energy_conserv);
    ui->add_k->setText(QString("%1").arg(engine->settings.add_k));
    ui->min_k->setText(QString("%1").arg(engine->settings.min_k));
    ui->max_k->setText(QString("%1").arg(engine->settings.max_k));

    ui->set_k->setEnabled(!engine->settings.k_energy_conserv);
    ui->add_k->setEnabled(engine->settings.k_energy_conserv);
    ui->max_k->setEnabled(engine->settings.k_energy_conserv);
    ui->min_k->setEnabled(engine->settings.k_energy_conserv);
}
