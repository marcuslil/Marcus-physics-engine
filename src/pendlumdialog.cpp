#include "pendlumdialog.h"
#include "ui_pendlumdialog.h"
#include <QDebug>

PendlumDialog::PendlumDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PendlumDialog)
{
    ui->setupUi(this);
}

PendlumDialog::~PendlumDialog()
{
    delete ui;
}

void PendlumDialog::accept()
{
    bool ok;
    mass=ui->mass->text().toDouble(&ok);
    if (!ok || mass==0.0) return;
    I=ui->I->text().toDouble(&ok);
    if (!ok || I==0.0) return;
    l=ui->mass->text().toDouble(&ok);
    if (!ok) return;
    v=ui->speed->text().toDouble(&ok);
    if (!ok) return;
    g=ui->g->text().toDouble(&ok);
    if (!ok) return;
    theta=ui->theta->text().toDouble(&ok);
    if (!ok) return;
    l=ui->length->text().toDouble(&ok);
    if (!ok) return;
    if (ui->simple->isChecked()) type=0;
    else if (ui->compound->isChecked()) type=1;
    else if (ui->string->isChecked()) type=2;
    else return;
    math=ui->math->isChecked();
    QDialog::accept();
}
