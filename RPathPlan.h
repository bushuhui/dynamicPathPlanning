
#ifndef __RPATH_PLAN_H__
#define __RPATH_PLAN_H__

#include "RMap.h"

namespace rtk {


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
enum RPathPlanTypes
{
    RPP_ASTAR,
    RPP_ASTAR_DYNAMIC
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class RPathPlan
{
public:
    RPathPlan();
    RPathPlan(RMap *map);
    virtual ~RPathPlan();

    int setMap(RMap *map);
    int getMap(RMap *map);

    int setStart(int x, int y);
    int getStart(int *px, int *py);
    int setEnd(int x, int y);
    int getEnd(int *px, int *py);

    int    getPath(RPath *p);
    RPath* getPath(void);

    virtual int pathPlan(void) = 0;
    int pathPlan(int sx, int sy, int ex, int ey);

    virtual int planBeg(void);
    virtual int planStep(void);
    virtual int planStepBackward(void);

protected:
    RMap    *m_map;                             // map

    int     m_startX, m_startY;                 // start point
    int     m_endX, m_endY;                     // end point

    int     m_robX, m_robY;                     // current position
    double  m_robT;                             // current angle
    int     m_robStep;                          // step index
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
RPathPlan* create_PathPlanObj(RPathPlanTypes pp_t);


} // end of namespace rtk

#endif // end of __RPATH_PLAN_H__
