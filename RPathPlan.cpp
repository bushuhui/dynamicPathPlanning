
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "RPathPlan.h"

#include "RPathPlan_astar.h"
#include "RPathPlan_astarDyn.h"

namespace  rtk {



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

RPathPlan::RPathPlan()
{
    m_map = NULL;

    m_startX = -1;
    m_startY = -1;
    m_endX   = -1;
    m_endY   = -1;

    m_robX   = -1;
    m_robY   = -1;
    m_robT   = 0;
}

RPathPlan::RPathPlan(RMap *map)
{
    m_map = map;

    map->getStart(&m_startX, &m_startY);
    map->getEnd(&m_endX, &m_endY);

    m_robX   = -1;
    m_robY   = -1;
    m_robT   = 0;
}

RPathPlan::~RPathPlan()
{

}

int RPathPlan::setMap(RMap *map)
{
    m_map = map;

    map->getStart(&m_startX, &m_startY);
    map->getEnd(&m_endX, &m_endY);

    return 0;
}

int RPathPlan::getMap(RMap *map)
{
    map = m_map;

    return 0;
}


int RPathPlan::setStart(int x, int y)
{
    m_startX = x;
    m_startY = y;

    return 0;
}

int RPathPlan::getStart(int *px, int *py)
{
    *px = m_startX;
    *py = m_startY;

    return 0;
}

int RPathPlan::setEnd(int x, int y)
{
    m_endX = x;
    m_endY = y;

    return 0;
}

int RPathPlan::getEnd(int *px, int *py)
{
    *px = m_endX;
    *py = m_endY;

    return 0;
}

int RPathPlan::getPath(RPath *p)
{
    *p = *(m_map->getPath());
    return 0;
}

RPath* RPathPlan::getPath(void)
{
    return m_map->getPath();
}




int RPathPlan::pathPlan(int sx, int sy, int ex, int ey)
{
    m_startX = sx;
    m_startY = sy;
    m_endX   = ex;
    m_endY   = ey;

    return pathPlan();
}


int RPathPlan::planBeg(void)
{
    printf("ERR: RPathPlan::plangBeg not implemented yet!\n");
    return -1;
}

int RPathPlan::planStep(void)
{
    printf("ERR: RPathPlan::plangStep not implemented yet!\n");
    return -1;
}

int RPathPlan::planStepBackward(void)
{
    printf("ERR: RPathPlan::planStepBackward not implemented yet!\n");
    return -1;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

RPathPlan* create_PathPlanObj(RPathPlanTypes pp_t)
{
    RPathPlan *pp = NULL;

    switch(pp_t) {
    case RPP_ASTAR:
        pp = new RPathPlan_astar;
        break;
    case RPP_ASTAR_DYNAMIC:
        pp = new RPathPlan_astarDyn;
        break;
    default:
        printf("ERR: create_PathPlanObj can not create given type: %d\n", pp_t);
        break;
    }

    return pp;
}

}
