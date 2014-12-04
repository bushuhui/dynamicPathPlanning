
#ifndef __RRANGESCAN_H__
#define __RRANGESCAN_H__

#include <vector>

#include "RMap.h"

namespace rtk {


#define RRANGESCAN_INF      9e99    // object in inifity


struct RRangeScanResItem
{
    double      dist, ang;          // distance, angle
    double      t_e;                // angle error
    int         ix, iy;             // map position
};

typedef std::vector<RRangeScanResItem> RRangeScanRes;

class RRangeScan
{
public:
    RRangeScan();
    RRangeScan(RMap *map);
    ~RRangeScan();

    int     setMap(RMap *map);
    RMap*   getMap(void);


    int     setRobot(double x, double y, double t);
    int     getRobot(double *x, double *y, double *t);

    int     setParam(double sr, double sa, double sar);

    int     setScanRange(double sr);
    double  getScanRange(void);
    int     setScanAngle(double sa);
    double  getScanAngle(void);
    int     setScanAngRes(double sar);
    double  getScanAngRes(void);

    int     scan(void);
    int     scan(double x, double y, double t);

    int     getScanRes(int *sn, RRangeScanResItem **sr);
    int     getScanRes(RRangeScanRes *res);
    int     setScanRes2Map(RMap *map, int v = RMAP_OBSTACLE);
    int     getFacedObstacle(double *d);


private:
    int     calc_t_idx(double x, double y, double &t2);

protected:
    RMap    *m_map;                 // map obj

    double  m_robX, m_robY;         // robot position
    double  m_robT;                 // robot angle

    double  m_scanRange;            // scan range (unit map cell)
    double  m_scanAngle;            // scan angle (-m_scanAngle ~ 0 ~ m_scanAngle)
    double  m_scanAngRes;           // scan angle resolution (in degree)
    int     m_scanAngN;             // scan angle number

    RRangeScanResItem  *m_scanRes;             // scan results
};

} // end of namespace rtk

#endif // end of __RRANGESCAN_H__
