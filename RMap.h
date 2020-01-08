

#ifndef __RMAP_H__
#define __RMAP_H__

#include <stdio.h>
#include <stdint.h>

#include <vector>

namespace rtk {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef std::vector<uint32_t> RPath;

enum RMapItemType
{
    RMAP_FREE = 0,
    RMAP_OBSTACLE,
    RMAP_OBSTACLE_UNDESCOVERED,
    RMAP_OBSTACLE_SCANNED,
    RMAP_START_POINT,
    RMAP_END_POINT,
    RMAP_CURRENT_POSITION
};

class RMap
{
public:
    RMap();
    RMap(int nx, int ny);
    ~RMap();


    int     setSize(int mx, int my);
    int     getSizeX(void);
    int     getSizeY(void);


    int8_t* getMap(void);
    int     getMapCell(int ix, int iy);
    int     getMapCell(uint32_t i);
    int     setMapCell(int ix, int iy, int v);
    int     setMapCell(uint32_t i, int v);
    int     convMapCellValue(int v=RMAP_OBSTACLE_SCANNED, int v2=RMAP_OBSTACLE);
    int     clear(int setBoarder=1);

    int     setStart(int ix, int iy);
    int     getStart(int *px, int *py);
    int     setEnd(int ix, int iy);
    int     getEnd(int *px, int *py);

    int     setPathNode(int iStep, int ix, int iy);
    int     setPathNode(int iStep, uint32_t idx);
    int     getPathNode(int iStep);
    int     getPathNode(int iStep, int *ix, int *iy);
    int     getPathNode(int step, int *ix, int *iy, double *t);
    int     pushPathNode(int ix, int iy);
    int     pushPathNode(uint32_t idx);
    int     setPath(RPath *p);
    RPath*  getPath(void);
    int     getPath(RPath *p);
    int     getPathNodeNum(void);
    int     clearPath(void);


    int     pushTrackNode(int ix, int iy);
    int     pushTrackNode(uint32_t idx);
    int     setTrack(RPath *p);
    RPath*  getTrack(void);
    int     getTrack(RPath *p);
    int     getTrackNodeNum(void);
    int     clearTrack(void);


    int     setRobPos(int ix, int iy, double t);
    int     getRobPos(int *px, int *py, double *pt);

    int     deepCopy(RMap *map);

    int     save(const char *fname);
    int     load(const char *fname);

protected:
    int8_t      *m_map;
    int32_t     m_mx, m_my;

    int32_t     m_startX, m_startY;
    int32_t     m_endX,   m_endY;

    int32_t     m_robX, m_robY;
    double      m_robT;

    RPath       m_path;
    RPath       m_track;
};

} // end of namespace rtk

#endif // end of __RMAP_H__
