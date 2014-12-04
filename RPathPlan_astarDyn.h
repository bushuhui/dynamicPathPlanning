
#ifndef __RPATHPLAN_ASTARDYN_H__
#define __RPATHPLAN_ASTARDYN_H__

#include "RPathPlan.h"
#include "RRangeScan.h"

namespace rtk {

class RPathPlan_astarDyn : public RPathPlan
{
public:
    RPathPlan_astarDyn();
    ~RPathPlan_astarDyn();

    virtual int pathPlan(void);

    virtual int planBeg(void);
    virtual int planStep(void);
    virtual int planStepBackward(void);

protected:
    void        *m_dat;

    int         m_dynPlanState;

    RMap        *m_mapDyn;
    int         m_mapDynStep;

    RMap        *m_mapScan;
    RRangeScan  m_scan;
};

} // end of namespace rtk


#endif // end of __RPATHPLAN_ASTARDYN_H__
