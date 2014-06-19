#ifndef PENDLUMDIALOG_H
#define PENDLUMDIALOG_H

#include <QDialog>
#include <QAbstractButton>

namespace Ui {
class PendlumDialog;
}

class PendlumDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit PendlumDialog(QWidget *parent = 0);
    ~PendlumDialog();
    qreal mass,I,l,theta,v,g;
    bool math;
    int type;
    void accept();
private slots:
private:
    Ui::PendlumDialog *ui;
};

#endif // PENDLUMDIALOG_H
