#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QGraphicsView>

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit GraphicsView(QWidget *parent = 0);
    virtual void wheelEvent(QWheelEvent* event);

signals:

public slots:

};

#endif // GRAPHICSVIEW_H
