#include "graphicsview.h"
#include <QWheelEvent>

GraphicsView::GraphicsView(QWidget *parent) :
    QGraphicsView(parent)
{
}

void GraphicsView::wheelEvent(QWheelEvent *event)
{
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    const int degrees = event->delta() / 8;
    int steps = degrees / 15;

    QTransform m=transform();
    if (steps>0)
        m.scale(1.1,1.1);
    else
        m.scale(1.0/1.1,1.0/1.1);
    setTransform(m);
}

