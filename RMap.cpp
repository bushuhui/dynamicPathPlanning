
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "RMap.h"


namespace rtk {

#define RMAP_FILE_MAGIC     0x15432345


RMap::RMap()
{
    m_mx = 0;
    m_my = 0;
    m_map = NULL;

    m_startX = -1;
    m_startY = -1;
    m_endX   = -1;
    m_endY   = -1;

    m_robX = -1;
    m_robY = -1;
    m_robT = 0;

    m_path.clear();
    m_track.clear();
}

RMap::RMap(int nx, int ny)
{
    m_mx = nx;
    m_my = ny;
    m_map = new int8_t[nx*ny];
    for(int i=0; i<m_mx*m_my; i++) m_map[i] = 0;

    m_startX = -1;
    m_startY = -1;
    m_endX   = -1;
    m_endY   = -1;

    m_robX = -1;
    m_robY = -1;
    m_robT = 0;

    m_path.clear();
    m_track.clear();
}

RMap::~RMap()
{
    if( m_map != NULL ) {
        delete [] m_map;
        m_map = NULL;
    }

    m_mx = 0;
    m_my = 0;

    m_path.clear();
    m_track.clear();
}



int RMap::setSize(int mx, int my)
{
    if( m_map != NULL ) {
        delete [] m_map;
    }

    m_mx  = mx;
    m_my  = my;
    m_map = new int8_t[mx*my];

    for(int i=0; i<mx*my; i++) m_map[i] = 0;

    return 0;
}

int RMap::getSizeX(void)
{
    return m_mx;
}

int RMap::getSizeY(void)
{
    return m_my;
}



int8_t* RMap::getMap(void)
{
    return m_map;
}

int RMap::getMapCell(int ix, int iy)
{
    if( ix < 0 || ix > m_mx || iy < 0 || iy > m_my )
        return -1;
    else
        return m_map[iy*m_mx+ix];
}

int RMap::getMapCell(uint32_t i)
{
    if( i < 0 || i > m_mx*m_my )
        return -1;
    else
        return m_map[i];
}

int RMap::setMapCell(int ix, int iy, int v)
{
    if( ix < 0 || ix > m_mx || iy < 0 || iy > m_my )
        return -1;
    else
        m_map[iy*m_mx+ix] = v;

    return 0;
}

int RMap::setMapCell(uint32_t i, int v)
{
    if( i < 0 || i > m_mx*m_my )
        return -1;
    else
        m_map[i] = v;

    return 0;
}

int RMap::convMapCellValue(int v, int v2)
{
    uint32_t    i, n;

    n = m_mx*m_my;
    for(i=0; i<n; i++) {
        if( m_map[i] == v ) m_map[i] = v2;
    }

    return 0;
}


int RMap::clear(int setBoarder)
{
    int i, ix, iy;

    if( m_map == NULL ) return -1;

    for(i=0; i<m_mx*m_my; i++) m_map[i] = RMAP_FREE;

    if( setBoarder ) {
        iy = 0;      for(ix=0; ix<m_mx; ix++) m_map[iy*m_mx+ix] = RMAP_OBSTACLE;
        iy = m_my-1; for(ix=0; ix<m_mx; ix++) m_map[iy*m_mx+ix] = RMAP_OBSTACLE;
        ix = 0;      for(iy=0; iy<m_my; iy++) m_map[iy*m_mx+ix] = RMAP_OBSTACLE;
        ix = m_mx-1; for(iy=0; iy<m_my; iy++) m_map[iy*m_mx+ix] = RMAP_OBSTACLE;
    }

    m_startX = -1;
    m_startY = -1;
    m_endX   = -1;
    m_endY   = -1;

    m_path.clear();
    m_track.clear();

    return 0;
}




int RMap::setStart(int ix, int iy)
{
    m_startX = ix;
    m_startY = iy;

    return 0;
}

int RMap::getStart(int *px, int *py)
{
    *px = m_startX;
    *py = m_startY;

    return 0;
}

int RMap::setEnd(int ix, int iy)
{
    m_endX = ix;
    m_endY = iy;

    return 0;
}

int RMap::getEnd(int *px, int *py)
{
    *px = m_endX;
    *py = m_endY;

    return 0;
}





int RMap::setPathNode(int iStep, int ix, int iy)
{
    int n = m_path.size();

    if( iStep < 0 ) return -1;

    if( iStep <= n )
        m_path[iStep] = iy*m_mx + ix;
    else
        return -2;
}

int RMap::setPathNode(int iStep, uint32_t idx)
{
    int n = m_path.size();

    if( iStep < 0 ) return -1;

    if( iStep <= n )
        m_path[iStep] = idx;
    else
        return -2;
}

int RMap::getPathNode(int iStep)
{
    int n = m_path.size();

    if( iStep < 0 || iStep >= n ) return -1;

    return m_path[iStep];
}

int RMap::getPathNode(int iStep, int *ix, int *iy)
{
    int n = m_path.size();
    int idx;

    if( iStep < 0 || iStep >= n ) return -1;

    idx = m_path[iStep];
    *ix = idx % m_mx;
    *iy = idx / m_mx;

    return 0;
}

int RMap::getPathNode(int step, int *ix, int *iy, double *t)
{
    double      dx, dy;
    uint32_t    idx1, idx2;
    int         ix1, iy1, ix2, iy2;


    if( m_path.size() <= 1 )
        return -1;
    if( step < 0 || step >= m_path.size() )
        return -1;

    if( step == m_path.size()-1 ) {
        idx1 = m_path[step-1];
        idx2 = m_path[step];

        *ix = idx2 % m_mx;
        *iy = idx2 / m_mx;
    } else {
        idx1 = m_path[step];
        idx2 = m_path[step+1];

        *ix = idx1 % m_mx;
        *iy = idx1 / m_mx;
    }

    ix1 = idx1 % m_mx;
    iy1 = idx1 / m_mx;
    ix2 = idx2 % m_mx;
    iy2 = idx2 / m_mx;

    dx = ix2 - ix1;
    dy = iy2 - iy1;
    *t = atan2(dy, dx);

    return 0;
}


int RMap::pushPathNode(int ix, int iy)
{
    m_path.push_back(iy*m_mx+ix);
    return 0;
}

int RMap::pushPathNode(uint32_t idx)
{
    m_path.push_back(idx);
    return 0;
}

int RMap::setPath(RPath *p)
{
    m_path = *p;
    return 0;
}

RPath* RMap::getPath(void)
{
    return &m_path;
}

int RMap::getPath(RPath *p)
{
    *p = m_path;
    return 0;
}


int RMap::getPathNodeNum(void)
{
    return m_path.size();
}

int RMap::clearPath(void)
{
    m_path.clear();
    return 0;
}




int RMap::pushTrackNode(int ix, int iy)
{
    m_track.push_back(iy*m_mx+ix);

    return 0;
}

int RMap::pushTrackNode(uint32_t idx)
{
    m_track.push_back(idx);

    return 0;
}

int RMap::setTrack(RPath *p)
{
    m_track = *p;

    return 0;
}

RPath* RMap::getTrack(void)
{
    return &m_track;
}

int RMap::getTrack(RPath *p)
{
    *p = m_track;

    return 0;
}

int RMap::getTrackNodeNum(void)
{
    return m_track.size();
}

int RMap::clearTrack(void)
{
    m_track.clear();

    return 0;
}




int RMap::setRobPos(int ix, int iy, double t)
{
    m_robX = ix;
    m_robY = iy;
    m_robT = t;

    return 0;
}

int RMap::getRobPos(int *px, int *py, double *pt)
{
    *px = m_robX;
    *py = m_robY;
    *pt = m_robT;

    return 0;
}




int RMap::deepCopy(RMap *map)
{
    uint32_t        n;

    map->m_mx = m_mx;
    map->m_my = m_my;
    n = m_mx*m_my;
    map->m_map = new int8_t[n];
    memcpy(map->m_map, m_map, sizeof(int8_t)*n);

    map->m_startX = m_startX;
    map->m_startY = m_startY;
    map->m_endX   = m_endX;
    map->m_endY   = m_endY;

    map->m_robX   = m_robX;
    map->m_robY   = m_robY;
    map->m_robT   = m_robT;

    map->m_path   = m_path;
    map->m_track  = m_track;

    return 0;
}




int RMap::save(const char *fname)
{
    FILE        *fp = NULL;
    int         res, ret = -1;
    uint32_t    f_magic;


    // open file
    fp = fopen(fname, "wb");
    if( fp == NULL ) {
        printf("ERR: failed to open file %s!\n", fname);
        return -1;
    }

    // check file magic number
    f_magic = RMAP_FILE_MAGIC;
    res = fwrite(&f_magic, sizeof(uint32_t), 1, fp);

    // write map nx, ny
    res = fwrite(&m_mx, sizeof(int32_t), 1, fp);
    res = fwrite(&m_my, sizeof(int32_t), 1, fp);

    // write map
    res = fwrite(m_map, sizeof(int8_t), m_mx*m_my, fp);

    // write start,end point
    res = fwrite(&m_startX, sizeof(int32_t), 1, fp);
    res = fwrite(&m_startY, sizeof(int32_t), 1, fp);

    res = fwrite(&m_endX, sizeof(int32_t), 1, fp);
    res = fwrite(&m_endY, sizeof(int32_t), 1, fp);

    // read path node
    uint32_t    pn;
    uint32_t    *md;

    pn = m_path.size();
    res = fwrite(&pn, sizeof(uint32_t), 1, fp);

    if( pn > 0 ) {
        md = (uint32_t*) m_path.data();
        res = fwrite(md, sizeof(uint32_t), pn, fp);
    }

    ret = 0;

RMAP_SAVE_RET:
    fclose(fp);

    return ret;
}

int RMap::load(const char *fname)
{
    FILE        *fp = NULL;
    int         res, ret = -1;
    uint32_t    f_magic;


    // open file
    fp = fopen(fname, "rb");
    if( fp == NULL ) {
        printf("ERR: failed to open file %s!\n", fname);
        return -1;
    }

    // check file magic number
    res = fread(&f_magic, sizeof(uint32_t), 1, fp);
    if( f_magic != RMAP_FILE_MAGIC ) {
        printf("ERR: input file format is not correct! %s\n", fname);
        goto RMAP_LOAD_RET;
    }

    // read map nx, ny
    res = fread(&m_mx, sizeof(int32_t), 1, fp);
    res = fread(&m_my, sizeof(int32_t), 1, fp);

    // read map
    if( m_map != NULL ) delete [] m_map;
    m_map = new int8_t[m_mx*m_my];
    res = fread(m_map, sizeof(int8_t), m_mx*m_my, fp);

    // read start,end point
    res = fread(&m_startX, sizeof(int32_t), 1, fp);
    res = fread(&m_startY, sizeof(int32_t), 1, fp);

    res = fread(&m_endX, sizeof(int32_t), 1, fp);
    res = fread(&m_endY, sizeof(int32_t), 1, fp);

    // read path node
    uint32_t    pn;
    uint32_t    *md;
    res = fread(&pn, sizeof(uint32_t), 1, fp);

    m_path.clear();

    if( pn > 0 ) {
        m_path.resize(pn);
        md = (uint32_t*) m_path.data();
        res = fread(md, sizeof(uint32_t), pn, fp);
    }

    ret = 0;

RMAP_LOAD_RET:
    fclose(fp);

    return ret;
}

} // end of namespace rtk
