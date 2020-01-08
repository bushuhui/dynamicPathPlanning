
#ifndef __RPATHPLAN_ASTAR_H__
#define __RPATHPLAN_ASTAR_H__

#include "RPathPlan.h"
#include "RRangeScan.h"

namespace rtk {

class RPathPlan_astar : public RPathPlan
{
public:
    RPathPlan_astar();
    ~RPathPlan_astar();

    virtual int pathPlan(void);

    virtual int planBeg(void);
    virtual int planStep(void);
    virtual int planStepBackward(void);

protected:
    void        *m_dat;
    RRangeScan  m_scan;
};

} // end of namespace rtk


#endif // end of __RPATHPLAN_ASTAR_H__
