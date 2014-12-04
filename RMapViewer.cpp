
#include <stdio.h>
#include <math.h>

#include <QtCore>
#include <QtGui>
#include <QFileDialog>
#include <QObject>
#include <QMenu>
#include <QString>
#include <QApplication>

#include "RMap.h"
#include "RMapViewer.h"


namespace rtk {


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class RMapLayer : public QGraphicsItem
{
    friend class RMapViewer;

public:
    RMapLayer(RMapViewer *mv, RMap *map);

    int setMap(RMap *m);

    QRectF boundingRect() const;
    QPainterPath shape() const;

    void paint(QPainter *painter,
               const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

protected:
    RMap        *m_map;
    RMapViewer   *m_mv;

    int         m_bMousePressed;
    int         m_lstX, m_lstY;
};



RMapLayer::RMapLayer(RMapViewer *mv, RMap *map)
{
    m_map = map;
    m_mv  = mv;

    m_bMousePressed = 0;

    m_lstX = 0;
    m_lstY = 0;

    setCacheMode(DeviceCoordinateCache);
    setZValue(-1);
}

int RMapLayer::setMap(RMap *m)
{
    m_map = m;
    this->update();

    return 0;
}

QRectF RMapLayer::boundingRect() const
{
    qreal adjust = m_mv->m_mapMagin;

    return QRectF(0-adjust, 0-adjust,
                  m_map->getSizeX()+2*adjust, m_map->getSizeY()+2*adjust);
}

QPainterPath RMapLayer::shape() const
{
    QPainterPath path;

    path.addRect(0, 0, m_map->getSizeX(), m_map->getSizeY());
    return path;
}

void RMapLayer::paint(QPainter *painter,
                      const QStyleOptionGraphicsItem *option,
                      QWidget *widget)
{
    int         mx, my;
    uint32_t    i;
    int         ix, iy;
    qreal       dx, dy, x1, y1, x2, y2;
    int8_t      v;

    qreal       borderWidth = 1.2;

    mx = m_map->getSizeX();
    my = m_map->getSizeY();

    x1 = 0;
    y1 = 0;
    dx = 1;
    dy = 1;

    // draw shadow
    {
        qreal shadowShift = mx/50;

        QRectF  rect(shadowShift-borderWidth/2, shadowShift-borderWidth/2,
                    mx+borderWidth, my+borderWidth);
        QColor  cl(220, 220, 220);

        painter->setPen(QPen(cl, borderWidth, Qt::SolidLine, Qt::RoundCap));
        painter->setBrush(cl);
        //painter->fillRect(rect, cl);
        painter->drawRect(rect);

        painter->setBrush(Qt::white);
        painter->fillRect(QRectF(0, 0, mx, my), QColor(255, 255, 255));
    }

    // draw boarder
    {
        painter->setPen(QPen(QColor(0, 100, 100), borderWidth,
                             Qt::SolidLine, Qt::RoundCap));
        painter->drawRect(QRectF(-borderWidth/2, -borderWidth/2,
                                 mx+borderWidth, my+borderWidth));
    }

    // draw grid
    if( m_mv->getDrawGrid() ) {
        painter->setPen(QColor(240, 240, 240));

        y2 = my;
        for(i=0; i<=mx; i++) {
            x2 = x1 + i*dx;
            painter->drawLine(x2, y1, x2, y2);
        }

        x2 = mx;
        for(i=0; i<=my; i++) {
            y2 = y1 + i*dy;
            painter->drawLine(x1, y2, x2, y2);
        }
    }


    // draw cells
    for(i=0; i<mx*my; i++) {
        v = m_map->getMapCell(i);

        if( v == RMAP_FREE ) continue;

        ix = i % mx;
        iy = i / my;

        x2 = x1 + ix*dx;
        y2 = y1 + iy*dy;

        switch(v) {
        case RMAP_OBSTACLE:
            painter->fillRect(x2, y2, dx, dy, Qt::black);
            break;
        case RMAP_OBSTACLE_UNDESCOVERED:
            painter->fillRect(x2, y2, dx, dy, QColor(180, 180, 180));
            break;
        case RMAP_OBSTACLE_SCANNED:
            painter->fillRect(x2, y2, dx, dy, QColor(255, 0, 0));
            break;
        default:
            break;
        }
    }

    // draw path
    if( 1 ) {
        RPath           *p;
        int             pn;
        RPath::iterator ni;
        uint32_t        idx1, idx2;
        qreal           ix1, iy1, ix2, iy2;

        p = m_map->getPath();
        pn = p->size();

        qreal lineWidth = 0.3;
        painter->setPen(QPen(QColor(0, 255, 0), lineWidth,
                             Qt::SolidLine, Qt::RoundCap));

        i = 0;
        for(ni=p->begin(); ni!=p->end(); ni++) {
            idx1 = (*ni);
            if( i == 0 ) idx2 = idx1;

            if( idx1 != idx2 ) {
                ix1 = idx1 % mx + 0.5;
                iy1 = idx1 / mx + 0.5;
                ix2 = idx2 % mx + 0.5;
                iy2 = idx2 / mx + 0.5;

                painter->drawLine(QLineF(ix1, iy1, ix2, iy2));
            }

            idx2 = idx1;
            i++;
        }
    }

    // draw track path
    if( 1 ) {
        RPath           *p;
        int             pn;
        RPath::iterator ni;
        uint32_t        idx1, idx2;
        qreal           ix1, iy1, ix2, iy2;

        p  = m_map->getTrack();
        pn = p->size();

        qreal lineWidth = 0.3;
        painter->setPen(QPen(QColor(0, 255, 255), lineWidth,
                             Qt::SolidLine, Qt::RoundCap));

        i = 0;
        for(ni=p->begin(); ni!=p->end(); ni++) {
            idx1 = (*ni);
            if( i == 0 ) idx2 = idx1;

            if( idx1 != idx2 ) {
                ix1 = idx1 % mx + 0.5;
                iy1 = idx1 / mx + 0.5;
                ix2 = idx2 % mx + 0.5;
                iy2 = idx2 / mx + 0.5;

                painter->drawLine(QLineF(ix1, iy1, ix2, iy2));
            }

            idx2 = idx1;
            i++;
        }
    }


    // draw start,end point
    m_map->getStart(&ix, &iy);
    if( ix >= 0 && ix < mx && iy >= 0 && iy < my ) {
        x2 = x1 + ix*dx;
        y2 = y1 + iy*dy;

        painter->fillRect(x2, y2, dx, dy, QColor(200, 0, 0));
    }

    m_map->getEnd(&ix, &iy);
    if( ix >= 0 && ix < mx && iy >= 0 && iy < my ) {
        x2 = x1 + ix*dx;
        y2 = y1 + iy*dy;

        painter->fillRect(x2, y2, dx, dy, QColor(0, 0, 255));
    }

    // draw current position
    {
        double  t;

        m_map->getRobPos(&ix, &iy, &t);

        if( ix >= 0 && ix < mx && iy >= 0 && iy < my ) {
            x2 = x1 + ix*dx;
            y2 = y1 + iy*dy;

            painter->setPen(QPen(QColor(255, 0, 255), 0.1, Qt::NoPen));
            painter->setBrush(QColor(255, 0, 255));
            painter->drawEllipse(QRectF(x2, y2, dx, dy));
            //painter->fillRect(x2, y2, dx, dy, QColor(255, 0, 255));
        }
    }
}


void RMapLayer::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QPointF     pt;
    float       cx, cy;
    int         ix, iy;
    int         nx, ny;
    uint32_t    idx;

    pt = this->pos();
    cx = event->scenePos().x() - pt.x();
    cy = event->scenePos().y() - pt.y();

    nx = m_map->getSizeX();
    ny = m_map->getSizeY();
    ix = (int)( cx );
    iy = (int)( cy );
    idx = iy*nx + ix;

    if( event->button() == Qt::LeftButton ) {
        if( m_mv->m_mapEdit ) {
            if( m_mv->m_mapEditDrawClear ) m_map->setMapCell(idx, 1);
            else                           m_map->setMapCell(idx, 0);
        }

        m_bMousePressed = 1;
    } else if( event->button() == Qt::RightButton ) {
        m_lstX = ix;
        m_lstY = iy;

        m_mv->m_popupMenu->popup(event->lastScreenPos());
    }

    update();
}

void RMapLayer::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    m_bMousePressed = 0;
}

void RMapLayer::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    static uint32_t  last_pos = 0;

    if( m_bMousePressed ) {
        QPointF     pt;
        float       cx, cy;
        int         ix, iy;
        int         nx, ny;
        uint32_t    idx;

        pt = this->pos();
        cx = event->scenePos().x() - pt.x();
        cy = event->scenePos().y() - pt.y();

        nx = m_map->getSizeX();
        ny = m_map->getSizeY();
        ix = (int)( cx );
        iy = (int)( cy );
        idx = iy*nx + ix;

        if( m_mv->m_mapEdit ) {
            if( m_mv->m_mapEditDrawClear ) m_map->setMapCell(idx, 1);
            else                           m_map->setMapCell(idx, 0);

            update();
        }
    }
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

RMapViewer::RMapViewer(QWidget *parent)
    : QGraphicsView(parent)
{
    // set variables
    m_mapEdit           = 0;
    m_mapEditDrawClear  = 1;

    // init map data
    m_map = new RMap(100, 100);
    m_map->clear();


    m_mapMagin = m_map->getSizeX()/50;
    m_drawGrid = 1;

    //setDragMode(ScrollHandDrag);

    // create scene
    m_scene = new QGraphicsScene(this);
    m_scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    m_scene->setSceneRect(0-m_mapMagin, 0-m_mapMagin,
                          m_map->getSizeX()+2*m_mapMagin,
                          m_map->getSizeY()+2*m_mapMagin);
    setScene(m_scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);

    setTransformationAnchor(AnchorUnderMouse);
    scale(qreal(6), qreal(6));

    setMinimumSize(400, 400);
    setWindowTitle(tr("Path plan GUI"));

    m_ml = new RMapLayer(this, m_map);
    m_ml->setPos(0, 0);
    m_scene->addItem(m_ml);

    setupMenu();

    m_pathPlanType = RPP_ASTAR_DYNAMIC;
    m_pathPlan = NULL;

    m_dragMode = 0;
}

RMapViewer::~RMapViewer()
{
    if( m_map != NULL ) delete m_map;

    if( m_pathPlan != NULL ) delete m_pathPlan;
}

int RMapViewer::setMap(RMap *map)
{
    m_map = map;

    m_scene->setSceneRect(0-m_mapMagin, 0-m_mapMagin,
                          m_map->getSizeX()+2*m_mapMagin,
                          m_map->getSizeY()+2*m_mapMagin);
    m_ml->setMap(map);

    this->update();
}

int RMapViewer::setDrawGrid(int drawGrid)
{
    m_drawGrid = drawGrid;
    m_ml->update();

    return 0;
}

int RMapViewer::getDrawGrid(void)
{
    return m_drawGrid;
}


void RMapViewer::keyPressEvent(QKeyEvent *event)
{

    switch (event->key()) {
    case Qt::Key_G:
        if( m_drawGrid == 0 )
            setDrawGrid(1);
        else
            setDrawGrid(0);
        break;


    case Qt::Key_E:
        if( m_mapEdit ) m_mapEdit = 0;
        else            m_mapEdit = 1;
        break;

    case Qt::Key_D:
        if( m_mapEditDrawClear ) m_mapEditDrawClear = 0;
        else                     m_mapEditDrawClear = 1;
        break;


    case Qt::Key_C:
        actMapClearRoute();
        break;
    case Qt::Key_X:
        actMapClear();
        break;

    case Qt::Key_Z:
        {
            int         mx, my;
            int         ix, iy;
            uint32_t    i;
            int8_t      *mp;

            mx = m_map->getSizeX();
            my = m_map->getSizeY();
            mp = m_map->getMap();

            if( 0 ) {
                iy = 0;      for(ix=0; ix<mx; ix++) mp[iy*mx+ix] = RMAP_OBSTACLE;
                iy = my-1;   for(ix=0; ix<mx; ix++) mp[iy*mx+ix] = RMAP_OBSTACLE;
                ix = 0;      for(iy=0; iy<my; iy++) mp[iy*mx+ix] = RMAP_OBSTACLE;
                ix = mx-1;   for(iy=0; iy<my; iy++) mp[iy*mx+ix] = RMAP_OBSTACLE;
            }

            if( 0 ) {
                for(i=0; i<mx*my; i++)
                    if( mp[i] == RMAP_OBSTACLE_SCANNED )
                        mp[i] = RMAP_OBSTACLE;
            }

            if( 1 ) {
                FILE        *fp = NULL;
                uint32_t    w, h;
                int8_t      *pm;

                // get file name
                QString fileName = QFileDialog::getOpenFileName(
                    this, tr("Load raw map"), ".", tr("Map Files (*.*)"));

                // load raw map
                fp = fopen(fileName.toStdString().c_str(), "rb");
                if( fp == NULL ) {
                    printf("ERR: Failed to read file: %s\n", fileName.toStdString().c_str());
                    break;
                }

                i = fread(&w, sizeof(uint32_t), 1, fp);
                i = fread(&h, sizeof(uint32_t), 1, fp);

                m_map->setSize(w, h);
                pm = m_map->getMap();

                i = fread(pm, sizeof(int8_t), w*h, fp);
                fclose(fp);

                m_scene->setSceneRect(0-m_mapMagin, 0-m_mapMagin,
                          m_map->getSizeX()+2*m_mapMagin,
                          m_map->getSizeY()+2*m_mapMagin);

                update();
            }

            m_ml->update();

            break;
        }


    case Qt::Key_S:
        actMapSave();
        break;
    case Qt::Key_L:
        actMapLoad();
        break;

    case Qt::Key_P:
        actPathPlanning();
        break;


    case Qt::Key_B:
    {
        if( m_pathPlan == NULL ) {
            m_pathPlan = create_PathPlanObj(m_pathPlanType);
        }

        m_pathPlan->setMap(m_map);
        m_pathPlan->planBeg();

        m_ml->update();
    }
    case Qt::Key_N:
    {
        if( m_pathPlan == NULL ) break;

        m_pathPlan->planStep();

        m_ml->update();

        break;
    }
    case Qt::Key_M:
    {
        if( m_pathPlan == NULL ) break;

        m_pathPlan->planStepBackward();

        m_ml->update();

        break;
    }
    case Qt::Key_H:
    {
        int ret = 0;

        if( m_pathPlan == NULL ) {
            m_pathPlan = create_PathPlanObj(m_pathPlanType);
        }

        m_pathPlan->setMap(m_map);
        ret = m_pathPlan->planBeg();

        m_ml->update();
        qApp->processEvents();

        while( ret == 0 ) {
            ret = m_pathPlan->planStep();
            m_ml->update();
            qApp->processEvents();
        }

        break;
    }




    case Qt::Key_Up:
        this->moveView(0, -1);
        break;
    case Qt::Key_Down:
        this->moveView(0, 1);
        break;
    case Qt::Key_Left:
        this->moveView(-1, 0);
        break;
    case Qt::Key_Right:
        this->moveView(1, 0);
        break;

    case Qt::Key_Plus:
        zoomIn();
        break;
    case Qt::Key_Minus:
        zoomOut();
        break;


    case Qt::Key_Space:
    case Qt::Key_Enter:
        break;

    default:
        QGraphicsView::keyPressEvent(event);
    }
}


void RMapViewer::mousePressEvent(QMouseEvent *event)
{
    if( event->button() == Qt::LeftButton && m_mapEdit == 0 ) {
        setDragMode(QGraphicsView::ScrollHandDrag);
        m_dragLastX = event->pos().x();
        m_dragLastY = event->pos().y();

        m_dragMode = 1;
    }

    QGraphicsView::mousePressEvent(event);
}

void RMapViewer::mouseMoveEvent(QMouseEvent *event)
{
    if( m_dragMode ) {
        int     nx, ny, dx, dy;
        int     v;

        nx = event->pos().x();
        ny = event->pos().y();

        dx = nx - m_dragLastX;
        dy = ny - m_dragLastY;

        v = verticalScrollBar()->sliderPosition();
        v -= dy;
        verticalScrollBar()->setSliderPosition(v);

        v = horizontalScrollBar()->sliderPosition();
        v -= dx;
        horizontalScrollBar()->setSliderPosition(v);

        m_dragLastX = nx;
        m_dragLastY = ny;
    } else {
        QGraphicsView::mouseMoveEvent(event);
    }
}

void RMapViewer::mouseReleaseEvent(QMouseEvent *event)
{
    if( m_dragMode ) {
        setDragMode(QGraphicsView::NoDrag);
        m_dragMode = 0;
    } else {
        QGraphicsView::mouseReleaseEvent(event);
    }
}



void RMapViewer::wheelEvent(QWheelEvent *event)
{
    scaleView(pow((double)2, -event->delta() / 900.0));
}




void RMapViewer::scaleView(qreal scaleFactor)
{
    qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
    //printf("factor = %f\n", factor);

    if (factor < 0.5 || factor > 100)
        return;

    scale(scaleFactor, scaleFactor);
}

void RMapViewer::moveView(qreal dx, qreal dy)
{
    int     v, v_min, v_max;

    qreal factor = transform().scale(1.0, 1.0).mapRect(QRectF(0, 0, 1, 1)).width();
    //printf("factor = %f\n", factor);

    if( fabs(dx) > 1e-6 ) {
        if( horizontalScrollBar() == NULL ) return;

        v     = horizontalScrollBar()->sliderPosition();
        v_min = horizontalScrollBar()->minimum();
        v_max = horizontalScrollBar()->maximum();

        //printf("v, v_min, v_max = %d, %d, %d\n", v, v_min, v_max);

        //v += dx*(v_max-v_min)*1.0/m_map->getSizeX();
        v += dx*(v_max-v_min)/(factor*2);
        if( v < v_min ) v = v_min;
        if( v > v_max ) v = v_max;

        horizontalScrollBar()->setSliderPosition(v);
    }

    if( fabs(dy) > 1e-6 ) {
        if( verticalScrollBar() == NULL ) return;

        v     = verticalScrollBar()->sliderPosition();
        v_min = verticalScrollBar()->minimum();
        v_max = verticalScrollBar()->maximum();

        //v += dy*(v_max-v_min)*1.0/m_map->getSizeY();
        v += dy*(v_max-v_min)/(factor*2);
        if( v < v_min ) v = v_min;
        if( v > v_max ) v = v_max;

        verticalScrollBar()->setSliderPosition(v);
    }
}


void RMapViewer::zoomIn()
{
    scaleView(qreal(1.05));
}

void RMapViewer::zoomOut()
{
    scaleView(1 / qreal(1.05));
}



int RMapViewer::setupMenu(void)
{
    // setup actions
    m_actMapEdit = new QAction(tr("Edit map"), this);
    m_actMapEdit->setCheckable(true);
    m_actMapEdit->setChecked(false);
    connect(m_actMapEdit, SIGNAL(triggered()), this, SLOT(actMapEdit()));

    m_actMapEditDrawClear = new QAction(tr("Draw/clear"), this);
    m_actMapEditDrawClear->setCheckable(true);
    m_actMapEditDrawClear->setChecked(true);
    m_actMapEditDrawClear->setDisabled(true);
    connect(m_actMapEditDrawClear, SIGNAL(triggered()), this, SLOT(actMapEditDrawClear()));

    m_actMapSetBegin = new QAction(tr("Set begin point"), this);
    connect(m_actMapSetBegin, SIGNAL(triggered()), this, SLOT(actMapSetBegin()));

    m_actMapSetEnd = new QAction(tr("Set end point"), this);
    connect(m_actMapSetEnd, SIGNAL(triggered()), this, SLOT(actMapSetEnd()));

    m_actMapClearRoute = new QAction(tr("Clear path"), this);
    connect(m_actMapClearRoute, SIGNAL(triggered()), this, SLOT(actMapClearRoute()));

    m_actMapClear = new QAction(tr("Clear all"), this);
    connect(m_actMapClear, SIGNAL(triggered()), this, SLOT(actMapClear()));

    m_actMapLoad = new QAction(tr("Load Map"), this);
    connect(m_actMapLoad, SIGNAL(triggered()), this, SLOT(actMapLoad()));

    m_actMapSave = new QAction(tr("Save Map"), this);
    connect(m_actMapSave, SIGNAL(triggered()), this, SLOT(actMapSave()));


    // setup menu
    m_popupMenu = new QMenu("Menu");
    m_popupMenu->addAction(m_actMapEdit);
    m_popupMenu->addAction(m_actMapEditDrawClear);

    m_popupMenu->addSeparator();

    m_popupMenu->addAction(m_actMapSetBegin);
    m_popupMenu->addAction(m_actMapSetEnd);

    m_popupMenu->addSeparator();

    m_popupMenu->addAction(m_actMapLoad);
    m_popupMenu->addAction(m_actMapSave);

    m_popupMenu->addSeparator();

    m_popupMenu->addAction(m_actMapClearRoute);
    m_popupMenu->addAction(m_actMapClear);

    return 0;
}

void RMapViewer::actMapEdit()
{
    m_mapEdit = m_actMapEdit->isChecked();

    if( m_mapEdit )
        m_actMapEditDrawClear->setDisabled(false);
    else
        m_actMapEditDrawClear->setDisabled(true);
}

void RMapViewer::actMapEditDrawClear()
{
    m_mapEditDrawClear = m_actMapEditDrawClear->isChecked();
}

void RMapViewer::actMapSetBegin()
{
    m_map->setStart(m_ml->m_lstX, m_ml->m_lstY);
    m_ml->update();
}

void RMapViewer::actMapSetEnd()
{
    m_map->setEnd(m_ml->m_lstX, m_ml->m_lstY);
    m_ml->update();
}

void RMapViewer::actMapClearRoute()
{
    m_map->clearPath();
    m_map->clearTrack();
    m_map->setRobPos(-1, -1, 0);
    m_map->convMapCellValue(RMAP_OBSTACLE_UNDESCOVERED, RMAP_OBSTACLE);
    m_map->convMapCellValue(RMAP_OBSTACLE_SCANNED, RMAP_OBSTACLE);
    m_ml->update();

    m_ml->update();
}

void RMapViewer::actMapClear()
{
    m_map->clear();
    m_map->clearPath();
    m_map->clearTrack();
    m_map->setRobPos(-1, -1, 0);
    m_map->convMapCellValue(RMAP_OBSTACLE_UNDESCOVERED, RMAP_OBSTACLE);
    m_map->convMapCellValue(RMAP_OBSTACLE_SCANNED, RMAP_OBSTACLE);

    m_ml->update();
}

void RMapViewer::actMapLoad()
{
    // get file name
    QString fileName = QFileDialog::getOpenFileName(
                this, tr("Load map"), "./map", tr("Map Files (*.map)"));

    m_map->clearPath();
    m_map->clearTrack();
    m_map->setRobPos(-1, -1, 0);

    // load map
    m_map->load(fileName.toStdString().c_str());

    m_mapMagin = m_map->getSizeX()/50;

    // update scene
    m_scene->setSceneRect(0-m_mapMagin, 0-m_mapMagin,
                          m_map->getSizeX()+2*m_mapMagin,
                          m_map->getSizeY()+2*m_mapMagin);

    // FIXME: why change scene rect didn't work correctly
    update();

    m_scene->update();
    // update canvas
    m_ml->update();

}

void RMapViewer::actMapSave()
{
    // get file name
    QString fileName = QFileDialog::getSaveFileName(
                this, tr("Save map"), "./map", tr("Map Files (*.map)"));

    // check file ext name
    QFileInfo *fi = new QFileInfo(fileName);
    if( fi->suffix().size() < 1 ) {
        fileName = fileName + ".map";
    }

    // save to file
    m_map->save(fileName.toStdString().c_str());
}

void RMapViewer::actPathPlanning(void)
{
    if( m_pathPlan == NULL )
        m_pathPlan = create_PathPlanObj(m_pathPlanType);

    m_pathPlan->setMap(m_map);
    m_pathPlan->pathPlan();

    m_ml->update();
}



} // end of namespace rtk
