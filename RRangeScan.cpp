
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "RRangeScan.h"


namespace rtk {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
inline double angle_pi2pi(double a)
{
    // convert to 0~2*pi
    if( fabs(a) > 2*M_PI ) {
        int n = (int)( a / (2*M_PI) );
        a = a - n*2*M_PI;
    }

    if( a >= M_PI ) {
        a = a - 2*M_PI;
    }

    if( a < -M_PI ) {
        a = a + 2*M_PI;
    }

    return a;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


RRangeScan::RRangeScan()
{
    m_map = NULL;

    m_scanRange  = 30;
    m_scanAngle  = 180.0/180.0*M_PI;
    m_scanAngRes = 1.0/180.0*M_PI;

    m_robX = 0;
    m_robY = 0;
    m_robT = 0;

    m_scanRes = NULL;
}

RRangeScan::RRangeScan(RMap *map)
{
    RRangeScan();

    m_map = map;
}

RRangeScan::~RRangeScan()
{
    if( m_scanRes != NULL ) {
        delete m_scanRes;
        m_scanRes = NULL;
    }

    m_map = NULL;
}


int RRangeScan::setMap(RMap *map)
{
    m_map = map;

    return 0;
}

RMap* RRangeScan::getMap(void)
{
    return m_map;
}


int RRangeScan::setRobot(double x, double y, double t)
{
    m_robX = x;
    m_robY = y;
    m_robT = t;

    return 0;
}

int RRangeScan::getRobot(double *x, double *y, double *t)
{
    *x = m_robX;
    *y = m_robY;
    *t = m_robT;

    return 0;
}

int RRangeScan::setParam(double sr, double sa, double sar)
{
    m_scanRange  = sr;
    m_scanAngle  = sa;
    m_scanAngRes = sar;

    return 0;
}

int RRangeScan::setScanRange(double sr)
{
    m_scanRange = sr;

    return 0;
}

double  RRangeScan::getScanRange(void)
{
    return m_scanRange;
}

int RRangeScan::setScanAngle(double sa)
{
    m_scanAngle = sa;

    return 0;
}

double RRangeScan::getScanAngle(void)
{
    return m_scanAngle;
}

int RRangeScan::setScanAngRes(double sar)
{
    m_scanAngRes = sar;

    return 0;
}

double RRangeScan::getScanAngRes(void)
{
    return m_scanAngRes;
}


int RRangeScan::calc_t_idx(double x, double y, double &t2)
{
    double  dx, dy, d;
    double  t;
    int     ti;

    dx = x - m_robX;
    dy = y - m_robY;
    d  = sqrt(dx*dx + dy*dy);
    t  = atan2(dy, dx);
    t2 = angle_pi2pi(t-m_robT);
    ti = (int)( (t2+m_scanAngle)/m_scanAngRes );

    return ti;
}


int RRangeScan::scan(void)
{
    int         mx, my;
    int8_t      *mp;

    int         ix1, iy1, ix2, iy2;
    int         i, j, k;
    uint32_t    idx;
    int         ti, ti_min, ti_max, ti_cent;

    double      fx, fy, d, t, t2, t2_cent, t_min, t_max, t_e;


    // calc parameters
    m_scanAngN = (int)( 2.0*m_scanAngle/m_scanAngRes );

    // create result array
    if( m_scanRes != NULL ) {
        delete [] m_scanRes;
    }

    m_scanRes = new RRangeScanResItem[m_scanAngN];
    for(i=0; i<m_scanAngN; i++) {
        m_scanRes[i].ix   = -1;
        m_scanRes[i].iy   = -1;
        m_scanRes[i].ang  = 0;
        m_scanRes[i].dist = RRANGESCAN_INF;
        m_scanRes[i].t_e  = RRANGESCAN_INF;
    }


    // get map data
    mx = m_map->getSizeX();
    my = m_map->getSizeY();
    mp = m_map->getMap();

    // get rough scan range
    ix1 = m_robX - m_scanRange;
    iy1 = m_robY - m_scanRange;
    ix2 = m_robX + m_scanRange;
    iy2 = m_robY + m_scanRange;

    if( ix1 < 0 ) ix1 = 0; if( ix1 >= mx ) ix1 = mx-1;
    if( iy1 < 0 ) iy1 = 0; if( iy1 >= my ) iy1 = my-1;
    if( ix2 < 0 ) ix2 = 0; if( ix2 >= mx ) ix2 = mx-1;
    if( iy2 < 0 ) iy2 = 0; if( iy2 >= my ) iy2 = my-1;

    // for each cell in local map
    for(j=iy1; j<=iy2; j++) {
        for(i=ix1; i<=ix2; i++) {
            idx = j*mx + i;

            // check is obstacle
            if( ! (mp[idx] == RMAP_OBSTACLE ||
                   mp[idx] == RMAP_OBSTACLE_UNDESCOVERED ||
                   mp[idx] == RMAP_OBSTACLE_SCANNED ) )
                continue;

            // get dx, dy, distance, angle
            fx = i - m_robX;
            fy = j - m_robY;
            d  = sqrt(fx*fx + fy*fy);
            t  = atan2(fy, fx);

            if( d > m_scanRange ) continue;

            ti = calc_t_idx(i, j, t2);
            t2_cent = t2;
            ti_cent = ti;
            if( ti < 0 || ti >= m_scanAngN ) continue;

            // find t_min, t_max
            // FIXME: need better way
            ti_min =  99999;
            ti_max = -99999;

            ti = calc_t_idx(-0.5+i, -0.5+j, t2);
            if( ti == 0 && ti_cent > m_scanAngN/2 ) ti = m_scanAngN-1;
            if( ti < ti_min ) { ti_min = ti; t_min = t2; }
            if( ti > ti_max ) { ti_max = ti; t_max = t2; }

            ti = calc_t_idx(-0.5+i,  0.5+j, t2);
            if( ti == 0 && ti_cent > m_scanAngN/2 ) ti = m_scanAngN-1;
            if( ti < ti_min ) { ti_min = ti; t_min = t2; }
            if( ti > ti_max ) { ti_max = ti; t_max = t2; }

            ti = calc_t_idx( 0.5+i, -0.5+j, t2);
            if( ti == 0 && ti_cent > m_scanAngN/2 ) ti = m_scanAngN-1;
            if( ti < ti_min ) { ti_min = ti; t_min = t2; }
            if( ti > ti_max ) { ti_max = ti; t_max = t2; }

            ti = calc_t_idx( 0.5+i,  0.5+j, t2);
            if( ti == 0 && ti_cent > m_scanAngN/2 ) ti = m_scanAngN-1;
            if( ti < ti_min ) { ti_min = ti; t_min = t2; }
            if( ti > ti_max ) { ti_max = ti; t_max = t2; }

            if( ti_min < 0 ) ti_min = 0;
            if( ti_min >= m_scanAngN ) ti_min = m_scanAngN-1;

            if( ti_max < 0 ) ti_max = 0;
            if( ti_max >= m_scanAngN ) ti_max = m_scanAngN-1;

            /*
            printf("    ti_min, ti, ti_max, d = %3d %3d %3d, %f\n",
                   ti_min, ti_cent, ti_max, d);
            */

            if( (ti_max - ti_min) < m_scanAngN/2 ) {
                for(k=ti_min; k<=ti_max; k++) {
                    if( m_scanRes[k].dist > d ) {
                        m_scanRes[k].ang  = t2_cent;
                        m_scanRes[k].dist = d;
                        m_scanRes[k].ix   = i;
                        m_scanRes[k].iy   = j;
                    }
                }
            } else {
                for(k=0; k<=ti_min; k++) {
                    if( m_scanRes[k].dist > d ) {
                        m_scanRes[k].ang  = t2_cent;
                        m_scanRes[k].dist = d;
                        m_scanRes[k].ix   = i;
                        m_scanRes[k].iy   = j;
                    }
                }

                for(k=ti_max; k<m_scanAngN; k++) {
                    if( m_scanRes[k].dist > d ) {
                        m_scanRes[k].ang  = t2_cent;
                        m_scanRes[k].dist = d;
                        m_scanRes[k].ix   = i;
                        m_scanRes[k].iy   = j;
                    }
                }
            }
        }
    }

    return 0;
}

int RRangeScan::scan(double x, double y, double t)
{
    m_robX = x;
    m_robY = y;
    m_robT = t;

    return scan();
}


int RRangeScan::getScanRes(int *sn, RRangeScanResItem **sr)
{
    *sn = m_scanAngN;
    *sr = m_scanRes;

    return 0;
}

int RRangeScan::getScanRes(RRangeScanRes *res)
{
    int     i;

    if( m_scanRes == NULL ) {
        return -1;
    }

    res->clear();
    for(i=0; i<m_scanAngN; i++)
        res->push_back(m_scanRes[i]);

    return 0;
}

int RRangeScan::setScanRes2Map(RMap *map, int v)
{
    int         mx, my, ix, iy;
    uint32_t    i;
    double      d;
    int8_t      *mp;

    if( m_scanRes == NULL ) {
        return -1;
    }

    mx = map->getSizeX();
    my = map->getSizeY();
    mp = map->getMap();

    for(i=0; i<m_scanAngN; i++) {
        d = m_scanRes[i].dist;
        if( d == RRANGESCAN_INF ) continue;

        ix = m_scanRes[i].ix;
        iy = m_scanRes[i].iy;
        if( ix < 0 || iy < 0 ) continue;

        mp[iy*mx+ix] = v;
    }

    return 0;
}


int RRangeScan::getFacedObstacle(double *d)
{
    if( m_scanRes == NULL ) {
        return -1;
    }

    int ic = (int)(m_scanAngN / 2.0 + 0.5);
    *d = m_scanRes[ic].dist;

    return 0;
}


} // end of namespace rtk
