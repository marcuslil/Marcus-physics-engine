#include <QtGui/QApplication>
#include "mainwindow.h"
#include <QDebug>

bool point_cossed_line(QPointF l1_bef,QPointF l2_bef,QPointF p_bef,QPointF l1_aft,QPointF l2_aft,QPointF p_aft);

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);


 /*   QPointF l1_bef(0,0);
    QPointF l2_bef(10,0);
    QPointF p_bef(4,4);

    QPointF l1_aft(0,0);
    QPointF l2_aft(10,0);
    QPointF p_aft(4,-1);





    qDebug() <<     point_cossed_line(l1_bef,l2_bef,p_bef,l1_aft,l2_aft,p_aft);
*/

    MainWindow w;
    w.show();
    
    return a.exec();
}
