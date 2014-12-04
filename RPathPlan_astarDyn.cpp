
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <vector>


#include "RPathPlan_astar.h"
#include "RPathPlan_astarDyn.h"

namespace rtk {


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


RPathPlan_astarDyn::RPathPlan_astarDyn()
{
    m_dat     = NULL;
    m_mapDyn  = NULL;
    m_mapScan = NULL;

    m_dynPlanState = 0;
}

RPathPlan_astarDyn::~RPathPlan_astarDyn()
{
    if( m_dat != NULL ) {
        RPathPlan_astar *as = (RPathPlan_astar *) m_dat;

        delete as;

        m_dat = NULL;
    }

    if( m_mapDyn != NULL ) {
        delete m_mapDyn;
        m_mapDyn = NULL;
    }

    if( m_mapScan != NULL ) {
        delete m_mapScan;
        m_mapScan = NULL;
    }

    m_dynPlanState = 0;
}

int RPathPlan_astarDyn::pathPlan(void)
{
    RPathPlan_astar *as;
    int             ret;

    if( m_dat != NULL )
        as = (RPathPlan_astar *) m_dat;
    else {
        as = new RPathPlan_astar;
        m_dat = as;
    }

    m_map->clearPath();

    ret = as->setMap(m_map);
    ret = as->pathPlan();

    if( ret != 0 ) {
        printf("ERR: RPathPlan_astarDyn::pathPlan failed to plan path!\n");
        return -1;
    }

    return ret;
}


int RPathPlan_astarDyn::planBeg(void)
{
    int             ret = 0;

    RPathPlan_astar *as;

    int             mx, my;
    int             ix, iy, ix2, iy2;

    // get map size
    mx = m_map->getSizeX();
    my = m_map->getSizeY();
    if( mx <= 0 || my <= 0 ) {
        printf("ERR: input map is empty!\n");
        return -1;
    }

    // get start/end point
    m_map->getStart(&ix, &iy);
    if( ix == -1 || iy == -1 ) {
        printf("ERR: start point not defined!\n");
        return -2;
    }
    m_map->getEnd(&ix2, &iy2);
    if( ix2 == -1 || iy2 == -1 ) {
        printf("ERR: end point not defined!\n");
        return -2;
    }

    // get a* obj
    if( m_dat != NULL )
        as = (RPathPlan_astar *) m_dat;
    else {
        as = new RPathPlan_astar;
        m_dat = as;
    }

    // copy map
    if( m_mapDyn == NULL ) m_mapDyn = new RMap;
    m_map->deepCopy(m_mapDyn);

    if( m_mapScan == NULL ) m_mapScan = new RMap;
    m_map->deepCopy(m_mapScan);

    // set visual map data
    m_map->convMapCellValue(RMAP_OBSTACLE,          RMAP_OBSTACLE_UNDESCOVERED);
    m_map->convMapCellValue(RMAP_OBSTACLE_SCANNED,  RMAP_OBSTACLE_UNDESCOVERED);
    m_map->clearPath();
    m_map->clearTrack();

    // set dynamic plan map
    m_mapDyn->clear(0);
    m_mapDyn->clearPath();
    m_mapDyn->clearTrack();

    // set scan map
    m_mapScan->convMapCellValue(RMAP_OBSTACLE_UNDESCOVERED,  RMAP_OBSTACLE);
    m_mapScan->convMapCellValue(RMAP_OBSTACLE_SCANNED,       RMAP_OBSTACLE);
    m_mapScan->clearPath();
    m_mapScan->clearTrack();


    // set scanner's map & perform first scan
    m_scan.setMap(m_mapScan);
    m_mapScan->setRobPos(ix, iy, 0);

    m_scan.setRobot(ix, iy, 0);
    m_scan.scan();
    m_scan.setScanRes2Map(m_map,    RMAP_OBSTACLE_SCANNED);
    m_scan.setScanRes2Map(m_mapDyn, RMAP_OBSTACLE);

    // perform path plan
    m_mapDyn->setStart(ix, iy);
    m_mapDyn->setEnd(ix2, iy2);

    ret = as->setMap(m_mapDyn);
    if( ret != 0 ) {
        printf("ERR: RPathPlan_astarDyn::pathBeg failed to set dynamic map!\n");
        return -3;
    }

    ret = as->pathPlan();
    if( ret != 0 ) {
        printf("ERR: RPathPlan_astarDyn::pathBeg failed to do a* (%d)\n", ret);

        m_dynPlanState = 0;
        m_mapDynStep = 0;

        return -3;
    }

    as->getPath(m_map->getPath());
    m_map->getTrack()->push_back(iy*mx+ix);

    m_dynPlanState = 1;
    m_mapDynStep   = 0;

    return 0;
}

int RPathPlan_astarDyn::planStep(void)
{
    int         mx, my;
    double      fx, fy;
    int         ix, iy;
    double      d, t;

    RPathPlan_astar     *as;
    RPath               *rp;


    // check current state
    if( m_dynPlanState != 1 ) {
        printf("ERR: dynamic path plan not started!\n");
        return -1;
    }

    // get a* obj
    if( m_dat != NULL )
        as = (RPathPlan_astar *) m_dat;
    else {
        as = new RPathPlan_astar;
        m_dat = as;
    }

    // get map size
    mx = m_map->getSizeX();
    my = m_map->getSizeY();

    // get next position
    m_scan.getRobot(&fx, &fy, &t);
    ix = (int)(fx);
    iy = (int)(fy);

    // get faced obstalce distance
    m_scan.getFacedObstacle(&d);
    printf("faced distance: %g\n", d);

    if( d <= 4 ) {
        // perform path planning
        m_mapDyn->setStart(ix, iy);
        as->setMap(m_mapDyn);
        as->pathPlan();
        rp = m_mapDyn->getPath();

        if( rp->size() <= 0 ) {
            printf("ERR: a* failed to plan path!\n");
            m_dynPlanState = 0;
            return -2;
        }

        m_map->setPath(rp);
        m_mapDynStep = 0;

        // check whether it can go forward
        m_mapDyn->getPathNode(1, &ix, &iy, &t);

        if( m_mapScan->getMapCell(ix, iy) != RMAP_OBSTACLE ) {
            m_mapDynStep ++;

            m_scan.setRobot(ix, iy, t);
            m_scan.scan();
            m_scan.setScanRes2Map(m_map,    RMAP_OBSTACLE_SCANNED);
            m_scan.setScanRes2Map(m_mapDyn, RMAP_OBSTACLE);

            m_map->getTrack()->push_back(iy*mx+ix);
            m_map->setRobPos(ix, iy, t);
        }
    } else {
        rp = m_mapDyn->getPath();
        m_mapDynStep ++;

        if( m_mapDynStep >= rp->size() ) {
            // perform path planning
            m_mapDyn->setStart(ix, iy);
            as->setMap(m_mapDyn);
            as->pathPlan();
            rp = m_mapDyn->getPath();

            if( rp->size() <= 0 ) {
                printf("ERR: a* failed to plan path!\n");
                m_dynPlanState = 0;
                return -3;
            }

            m_map->setPath(rp);
            m_mapDynStep = 0;
        } else {
            // get next position
            m_mapDyn->getPathNode(m_mapDynStep, &ix, &iy, &t);

            m_map->setRobPos(ix, iy, t);
            m_map->convMapCellValue(RMAP_OBSTACLE_SCANNED, RMAP_OBSTACLE);

            // perform scan
            m_scan.setRobot(ix, iy, t);
            m_scan.scan();
            m_scan.setScanRes2Map(m_map, RMAP_OBSTACLE_SCANNED);
            m_scan.setScanRes2Map(m_mapDyn, RMAP_OBSTACLE);

            m_map->getTrack()->push_back(iy*mx+ix);
            m_map->setPath(rp);
        }
    }

    return 0;
}

int RPathPlan_astarDyn::planStepBackward(void)
{
    printf("ERR: RPathPlan_astarDyn::planStepBackward not implemented!\n");

    return -1;
}

} // end of namespace rtk
