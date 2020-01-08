
#ifndef __RMAPVIEWER_H__
#define __RMAPVIEWER_H__

#include <QtGui>
#include <QtCore>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QMenu>

#include "RMap.h"
#include "RPathPlan.h"
#include "RRangeScan.h"

namespace rtk {

class RMapLayer;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class RMapViewer : public QGraphicsView
{
    Q_OBJECT

    friend class RMapLayer;

public:
    RMapViewer(QWidget *parent = 0);
    ~RMapViewer();

    int setMap(RMap *map);

    int setDrawGrid(int drawGrid);
    int getDrawGrid(void);

public slots:
    void zoomIn();
    void zoomOut();

protected:
    void keyPressEvent(QKeyEvent *event);

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

    void wheelEvent(QWheelEvent *event);

    void scaleView(qreal scaleFactor);
    void moveView(qreal dx, qreal dy);

protected:
    QGraphicsScene  *m_scene;
    RMapLayer       *m_ml;


    int             m_mapMagin;             // margin of map boarder
    int             m_drawGrid;             // draw grid

    int             m_mapEdit;              // map edit
    int             m_mapEditDrawClear;     // map draw/clear

    int             m_dragMode;             // mouse drag mode
    int             m_dragLastX;
    int             m_dragLastY;

    RMap            *m_map;
    RPathPlanTypes  m_pathPlanType;
    RPathPlan       *m_pathPlan;



protected:
    QMenu       *m_popupMenu;

    QAction     *m_actMapEdit;
    QAction     *m_actMapEditDrawClear;
    QAction     *m_actMapSetBegin;
    QAction     *m_actMapSetEnd;
    QAction     *m_actMapClearRoute;
    QAction     *m_actMapClear;
    QAction     *m_actMapLoad;
    QAction     *m_actMapSave;

    QAction     *m_actPathPlanning;

    int         setupMenu(void);

public slots:
    void        actMapEdit(void);
    void        actMapEditDrawClear(void);
    void        actMapSetBegin(void);
    void        actMapSetEnd(void);
    void        actMapClearRoute(void);
    void        actMapClear(void);
    void        actMapLoad(void);
    void        actMapSave(void);

    void        actPathPlanning(void);
};


} // end of namespace rtk

#endif // end of __RMAPVIEWER_H__
