
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <vector>

#include "RPathPlan_astar.h"

namespace rtk {


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef struct AStarNode
{
    int         s_x;                    // 坐标(最终输出路径需要)
    int         s_y;

    int         s_g;                    // 起点到此点的距离( 由g和h可以得到f，此处f省略，f=g+h )
    int	        s_h;                    // 启发函数预测的此点到终点的距离

    int8_t      s_style;                // 结点类型：起始点，终点，障碍物

    AStarNode   *s_parent;              // 父节点

    int         in_closetable;          // 是否在close表中
    int         in_opentable;           // 是否在open表中
} *pAStarNode;


typedef std::vector<pAStarNode>     nodeTable;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct AStarData
{
public:
    AStarData();
    ~AStarData();

    int  setMap(RMap *map);
    int  findRoute();
    int  getPath(RMap *map);

private:
    int  open_table_push(AStarNode *pn);
    void open_table_print(void);

    void swap(int idx1, int idx2);
    void adjust_heap(int nIndex);
    int  insert_to_opentable(int x, int y,
                             pAStarNode curr_node,
                             pAStarNode end_node,
                             int w);
    void add_neighbors(pAStarNode curr_node, pAStarNode end_node);


private:
    RMap        *m_map;
    int         nx, ny;                 // map size


    AStarNode   *map_maze;              // 结点数组
    nodeTable   path_stack;             // 保存路径的栈

    pAStarNode  *open_table;            // open表
    int         open_table_size;        // open-list pre-allocated size
    int         open_node_count;        // open表中节点数量

    AStarNode   *start_node;            // 起始点
    AStarNode   *end_node;              // 结束点
    AStarNode   *curr_node;             // 当前点
};


AStarData::AStarData()
{
    map_maze = NULL;

    open_node_count  = 0;

    open_table_size = 2048;
    open_table = new pAStarNode[open_table_size];
}

AStarData::~AStarData()
{
    if( map_maze != NULL ) {
        delete [] map_maze;
        map_maze = NULL;
    }

    if( open_table != NULL ) {
        delete [] open_table;
        open_table      = NULL;
        open_table_size = 0;
    }

    path_stack.clear();
}



int AStarData::setMap(RMap *map)
{
    uint32_t    i;
    int8_t      *p;
    int         ix, iy;
    uint32_t    idx;

    m_map = map;

    nx = map->getSizeX();
    ny = map->getSizeY();
    p  = map->getMap();

    if( nx <= 0 || ny <= 0 ) {
        printf("AStarData::setMap input map size error! (%d %d)\n", nx, ny);
        return -1;
    }

    // create inner map
    if( map_maze != NULL ) delete [] map_maze;
    map_maze = new AStarNode[nx*ny];

    for(i=0; i<nx*ny; i++) {
        ix = i % nx;
        iy = i / nx;

        map_maze[i].s_g             = 0;
        map_maze[i].s_h             = 0;
        map_maze[i].in_closetable   = 0;
        map_maze[i].in_opentable    = 0;
        map_maze[i].s_style         = p[i];
        map_maze[i].s_x             = ix;
        map_maze[i].s_y             = iy;
        map_maze[i].s_parent        = NULL;
    }

    // set start/end point
    map->getStart(&ix, &iy);
    if( ix < 0   || iy < 0   ) return -1;
    if( ix >= nx || iy >= ny ) return -1;
    idx = iy*nx + ix;
    start_node = &(map_maze[idx]);

    map->getEnd(&ix, &iy);
    if( ix < 0   || iy < 0   ) return -1;
    if( ix >= nx || iy >= ny ) return -1;
    idx = iy*nx + ix;
    end_node = &(map_maze[idx]);

    // clear path stack
    path_stack.clear();

    return 0;
}

int AStarData::getPath(RMap *map)
{
    int     ix, iy;

    // clear old path
    map->clearPath();

    if( path_stack.size() == 0 ) return -1;

    // reverse the path
    nodeTable::reverse_iterator rit;

    for(rit=path_stack.rbegin(); rit!=path_stack.rend(); rit++) {
        ix = (*rit)->s_x;
        iy = (*rit)->s_y;
        map->pushPathNode(ix, iy);

        //printf("(%d,%d) -> ", ix, iy);
    }
    //printf("\n");

    return 0;
}

int AStarData::open_table_push(AStarNode *pn)
{
    int         ns, i;
    pAStarNode  *a;

    // adjust open_table size dynamically
    if( open_node_count+1 >= open_table_size ) {
         ns = open_table_size*2;
         a = new pAStarNode[ns];

         for(i=0; i<open_node_count; i++) a[i] = open_table[i];

         delete [] open_table;

         open_table = a;
         open_table_size = ns;
    }

    // push to table last
    open_table[open_node_count++] = pn;

    return 0;
}

void AStarData::open_table_print(void)
{
    int     i;

    printf("ot (%6d),", open_node_count);
    for(i=0; i<open_node_count; i++) {
        printf("  (%d %d - %d %d)",
               open_table[i]->s_x, open_table[i]->s_y,
               open_table[i]->s_g, open_table[i]->s_h);
    }
    printf("\n");
}


// 交换两个元素
void AStarData::swap(int idx1, int idx2)
{
    pAStarNode tmp   = open_table[idx1];
    open_table[idx1] = open_table[idx2];
    open_table[idx2] = tmp;
}

// 堆调整
void AStarData::adjust_heap(int nIndex)
{
    int curr   = nIndex;
    int child  = curr * 2 + 1;      // 得到左孩子idx( 下标从0开始，所有做孩子是curr*2+1 )
    int parent = ( curr - 1 ) / 2;  // 得到双亲idx

    if (nIndex < 0 || nIndex >= open_node_count)
        return;

    // 往下调整( 要比较左右孩子和cuur parent )
    while ( child < open_node_count ) {
        // 小根堆是双亲值小于孩子值
        if ( child + 1 < open_node_count &&
             open_table[child]->s_g + open_table[child]->s_h  >
             open_table[child+1]->s_g + open_table[child+1]->s_h ) {
            ++child; // 判断左右孩子大小
        }

        if (open_table[curr]->s_g + open_table[curr]->s_h <=
                open_table[child]->s_g + open_table[child]->s_h) {
            break;
        } else {
            swap(child, curr);	            // 交换节点
            curr = child;                   // 再判断当前孩子节点
            child = curr * 2 + 1;           // 再判断左孩子
        }
    }

    if (curr != nIndex)
        return;

    // 往上调整( 只需要比较curr child和parent )
    while (curr != 0) {
        if (open_table[curr]->s_g + open_table[curr]->s_h >=
                open_table[parent]->s_g + open_table[parent]->s_h) {
            break;
        } else {
            swap(curr, parent);
            curr   = parent;
            parent = (curr-1)/2;
        }
    }
}

// insert neighbor nodes to open_table
int AStarData::insert_to_opentable(
        int x, int y,
        pAStarNode curr_node,
        pAStarNode end_node,
        int w )
{
    int         i;
    pAStarNode  cn;

    // check range
    if( x < 0 || x >= nx || y < 0 || y >= ny )
        return -1;

    // check whether pass the obstacle
    {
        int         dx, dy, ix1, iy1, ix2, iy2;
        uint32_t    idx1, idx2;

        dx = x - curr_node->s_x;
        dy = y - curr_node->s_y;

        if( abs(dx) == 1 || abs(dy) == 1 ) {
            ix1 = x;
            iy1 = curr_node->s_y;
            idx1 = iy1*nx + ix1;

            ix2 = curr_node->s_x;
            iy2 = y;
            idx2 = iy2*nx + ix2;

            // if two diag nodes are obstacles then return
            if( map_maze[idx1].s_style == 1 && map_maze[idx2].s_style == 1 )
                return -1;
        }
    }

    cn = &(map_maze[y*nx + x]);

    if( cn->s_style != 1 ) {                                // 不是障碍物
        if ( !cn->in_closetable ) {                         // 不在闭表中
            if ( cn->in_opentable ) {
                // 在open表中
                // 需要判断是否是一条更优化的路径
                if ( cn->s_g > curr_node->s_g + w ) {       // 如果更优化
                    cn->s_g      = curr_node->s_g + w;
                    cn->s_parent = curr_node;

                    for ( i = 0; i < open_node_count; ++i ) {
                        if ( open_table[i]->s_x == cn->s_x &&
                             open_table[i]->s_y == cn->s_y ) {
                            break;
                        }
                    }

                    adjust_heap( i );                       // 下面调整点
                }
            } else {
                // 不在open中
                cn->s_g = curr_node->s_g + w;
                cn->s_h = abs(end_node->s_x - x ) + abs(end_node->s_y - y);
                cn->s_parent = curr_node;

                cn->in_opentable = 1;
                open_table_push(cn);
            }
        }
    }

    return 0;
}

// add 8 neighbor nodes
void AStarData::add_neighbors(pAStarNode curr_node, pAStarNode end_node)
{
    int x = curr_node->s_x;
    int y = curr_node->s_y;

    // add 8 neighbor nodes
    insert_to_opentable( x+1, y,   curr_node, end_node, 10 );
    insert_to_opentable( x-1, y,   curr_node, end_node, 10 );
    insert_to_opentable( x,   y+1, curr_node, end_node, 10 );
    insert_to_opentable( x,   y-1, curr_node, end_node, 10 );
    insert_to_opentable( x+1, y+1, curr_node, end_node, 14 );
    insert_to_opentable( x+1, y-1, curr_node, end_node, 14 );
    insert_to_opentable( x-1, y+1, curr_node, end_node, 14 );
    insert_to_opentable( x-1, y-1, curr_node, end_node, 14 );
}


int AStarData::findRoute(void)
{
    int     is_found;

    open_node_count  = 0;

    // push start point to open_table
    start_node->in_opentable = 1;
    open_table_push(start_node);

    start_node->s_g = 0;
    start_node->s_h = abs(end_node->s_x - start_node->s_x) +
                      abs(end_node->s_y - start_node->s_y);
    start_node->s_parent = NULL;

    is_found = 0;

    // if start point is end point then return
    if ( start_node->s_x == end_node->s_x && start_node->s_y == end_node->s_y ) {
        printf("ERR: start == end\n");
        return -1;
    }


    while( 1 ) {
        curr_node = open_table[0];                      // open表的第一个点一定是f值最小的点(通过堆排序得到的)
        open_table[0] = open_table[--open_node_count];  // 最后一个点放到第一个点，然后进行堆调整

        adjust_heap( 0 );                               // 调整堆

        curr_node->in_closetable = 1;                   // 已经在close表中了

        if ( curr_node->s_x == end_node->s_x &&
             curr_node->s_y == end_node->s_y ) {        // 终点在close中，结束
            is_found = 1;
            break;
        }

        add_neighbors(curr_node, end_node);             // add neighbor nodes

        if ( open_node_count == 0 ) {                   // 没有路径到达
            is_found = 0;
            break;
        }
    }

    if ( is_found ) {
        curr_node = end_node;

        while( curr_node ) {
            path_stack.push_back(curr_node);
            curr_node = curr_node->s_parent;
        }
    } else {
        printf("ERR: No route can be found!\n");
        return -1;
    }

    return 0;
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


RPathPlan_astar::RPathPlan_astar()
{
    m_dat = NULL;
}

RPathPlan_astar::~RPathPlan_astar()
{
    if( m_dat != NULL ) {
        AStarData *d = (AStarData *) m_dat;

        delete d;

        m_dat = NULL;
    }
}

int RPathPlan_astar::pathPlan(void)
{
    AStarData   *d;
    int         ret = 0;

    if( m_dat != NULL )
        d = (AStarData *) m_dat;
    else {
        d = new AStarData;
        m_dat = d;
    }

    m_map->clearPath();

    ret = d->setMap(m_map);
    if( ret != 0 ) {
        printf("ERR: RPathPlan_astar::pathPlan failed to set map! (%d)\n", ret);
        return -1;
    }
    ret = d->findRoute();
    ret = d->getPath(m_map);

    return ret;
}


int RPathPlan_astar::planBeg(void)
{
    AStarData   *d;
    int         ret;

    if( m_dat != NULL )
        d = (AStarData *) m_dat;
    else {
        d = new AStarData;
        m_dat = d;
    }

    // prepare map data
    m_map->convMapCellValue(RMAP_OBSTACLE_SCANNED,      RMAP_OBSTACLE);
    m_map->convMapCellValue(RMAP_OBSTACLE_UNDESCOVERED, RMAP_OBSTACLE);
    m_map->clearPath();

    // perform path plan
    ret = d->setMap(m_map);
    if( ret != 0 ) {
        printf("ERR: RPathPlan_astar::pathBeg failed to set map!\n");
        return -1;
    }

    ret = d->findRoute();
    ret = d->getPath(m_map);

    if( ret != 0 ) return ret;

    // get initial position
    m_robStep = 0;
    m_map->getPathNode(m_robStep, &m_robX, &m_robY, &m_robT);

    // scan environment
    m_scan.setMap(m_map);
    m_scan.setRobot(m_robX, m_robY, m_robT);
    m_scan.scan();

    // set results to map
    m_scan.setScanRes2Map(m_map, RMAP_OBSTACLE_SCANNED);

    return 0;
}

int RPathPlan_astar::planStep(void)
{
    // prepare map data
    m_map->convMapCellValue(RMAP_OBSTACLE_SCANNED, RMAP_OBSTACLE);

    m_robStep ++;
    if( m_robStep >= m_map->getPathNodeNum() ) {
        m_robStep = 0;
        return -1;
    }

    // get current position
    m_map->getPathNode(m_robStep, &m_robX, &m_robY, &m_robT);
    m_map->setRobPos(m_robX, m_robY, m_robT);

    // scan environment
    m_scan.setMap(m_map);
    m_scan.setRobot(m_robX, m_robY, m_robT);
    m_scan.scan();

    // set results to map
    m_scan.setScanRes2Map(m_map, RMAP_OBSTACLE_SCANNED);

    return 0;
}

int RPathPlan_astar::planStepBackward(void)
{
    // prepare map data
    m_map->convMapCellValue(RMAP_OBSTACLE_SCANNED, RMAP_OBSTACLE);

    m_robStep --;
    if( m_robStep < 0 ) {
        m_robStep = 0;
        return -1;
    }

    // get current position
    m_map->getPathNode(m_robStep, &m_robX, &m_robY, &m_robT);
    m_map->setRobPos(m_robX, m_robY, m_robT);

    // scan environment
    m_scan.setMap(m_map);
    m_scan.setRobot(m_robX, m_robY, m_robT);
    m_scan.scan();

    // set results to map
    m_scan.setScanRes2Map(m_map, RMAP_OBSTACLE_SCANNED);

    return 0;
}

} // end of namespace rtk
