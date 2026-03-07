#include "road.h"
#include <string.h> // 用于 memset

// BFS状态节点结构体
typedef struct {
    Point_t player;    // 玩家(车模)坐标
    Point_t box;       // 箱子坐标
    int16   parent_idx;// 父节点在队列中的索引，用于回溯路径
    Action_e action;   // 从父节点到达此节点所做的动作
} BfsState_t;

// 方向偏移数组: 上，下，左，右
static const int8 dx[4] = {0, 0, -1, 1};
static const int8 dy[4] = {-1, 1, 0, 0};
static const Action_e dir_act[4] = {ACT_UP, ACT_DOWN, ACT_LEFT, ACT_RIGHT};

// --- RT1064 内存充足，直接开静态大数组 ---
// 队列大小：可以根据实际情况调整，单箱子地图通常不会超过5000个状态
#define MAX_QUEUE_SIZE 8000 
static BfsState_t bfs_queue[MAX_QUEUE_SIZE];

// 访问标记数组 visited[player_y][player_x][box_y][box_x]
// 记录某个特定“人+箱子”的状态是否被搜索过，防止死循环 (16x12x16x12 bytes ≈ 36KB)
static uint8 visited[MAP_ROWS][MAP_COLS][MAP_ROWS][MAP_COLS];

/**
 * @brief 判断坐标是否在地图范围内且不为墙壁
 */
static uint8 Is_Valid_Move(const uint8 map[MAP_ROWS][MAP_COLS], int8 y, int8 x) {
    if (x < 0 || x >= MAP_COLS || y < 0 || y >= MAP_ROWS) {
        return 0; // 越界
    }
    if (map[y][x] == MAP_WALL) {
        return 0; // 撞墙
    }
    return 1;
}

/**
 * @brief 推箱子单箱子最短路径解算 (BFS核心逻辑)
 */
uint8 Algo_Sokoban_Solve(const uint8 map[MAP_ROWS][MAP_COLS], Point_t player_pos, Point_t box_pos, Point_t target_pos, PathResult_t *result) {
    int16 head = 0; // 队列头索引
    int16 tail = 0; // 队列尾索引
    
    // 初始化结果
    result->step_count = 0;
    result->is_success = 0;
    
    // 清空访问标记
    memset(visited, 0, sizeof(visited));
    
    // 初始化起点状态并入队
    bfs_queue[tail].player = player_pos;
    bfs_queue[tail].box = box_pos;
    bfs_queue[tail].parent_idx = -1;
    bfs_queue[tail].action = ACT_NONE;
    visited[player_pos.y][player_pos.x][box_pos.y][box_pos.x] = 1;
    tail++;
    
    // 开始 BFS
    while (head < tail && tail < MAX_QUEUE_SIZE) {
        BfsState_t current = bfs_queue[head];
        
        // 检查是否到达目标
        if (current.box.x == target_pos.x && current.box.y == target_pos.y) {
            // 到达目的地，开始回溯路径
            int16 curr_idx = head;
            uint16 temp_step = 0;
            Action_e temp_path[200];
            
            // 从目标点往回找，直到起始点
            while (bfs_queue[curr_idx].parent_idx != -1) {
                temp_path[temp_step++] = bfs_queue[curr_idx].action;
                curr_idx = bfs_queue[curr_idx].parent_idx;
            }
            
            // 倒序存入 result->path (因为回溯是反向的)
            result->step_count = temp_step;
            for (int i = 0; i < temp_step; i++) {
                result->path[i] = temp_path[temp_step - 1 - i];
            }
            
            result->is_success = 1;
            return 1; // 解算成功
        }
        
        // 尝试向4个方向移动
        for (int i = 0; i < 4; i++) {
            int8 next_py = current.player.y + dy[i];
            int8 next_px = current.player.x + dx[i];
            
            // 如果玩家试图移动的位置不合法(越界或墙)
            if (!Is_Valid_Move(map, next_py, next_px)) continue;
            
            Point_t next_box = current.box; // 默认箱子没动
            
            // 如果玩家试图移动的位置正好是箱子，说明要推箱子
            if (next_px == current.box.x && next_py == current.box.y) {
                // 计算箱子被推后的新位置
                next_box.y += dy[i];
                next_box.x += dx[i];
                
                // 检查箱子被推后的位置是否合法 (不能越界，不能是墙)
                if (!Is_Valid_Move(map, next_box.y, next_box.x)) {
                    continue; // 箱子推不动，放弃该方向
                }
            }
            
            // 检查这个新的 (玩家+箱子) 状态是否已经搜索过
            if (!visited[next_py][next_px][next_box.y][next_box.x]) {
                // 没搜索过，入队
                visited[next_py][next_px][next_box.y][next_box.x] = 1;
                
                // 防止队列溢出
                if (tail >= MAX_QUEUE_SIZE) break; 
                
                bfs_queue[tail].player.y = next_py;
                bfs_queue[tail].player.x = next_px;
                bfs_queue[tail].box = next_box;
                bfs_queue[tail].parent_idx = head;
                bfs_queue[tail].action = dir_act[i];
                tail++;
            }
        }
        head++; // 队头出队
    }
    
    return 0; // 遍历完都没找到，解算失败
}