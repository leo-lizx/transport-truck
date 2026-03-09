#include "algo_bfs_scout.h"
#include <string.h>

// 移动方向偏移量：上、下、左、右
static const int8 dx[4] = {0, 0, -1, 1};
static const int8 dy[4] = {-1, 1, 0, 0};

// 定义静态大数组防止单片机栈溢出 (16x12 = 192个节点，内存占用极小)
static Point_t bfs_queue[MAP_ROWS * MAP_COLS];
static Point_t parent_map[MAP_ROWS][MAP_COLS];
static uint8   visited[MAP_ROWS][MAP_COLS];

/**
 * @brief 判断坐标是否可通行（核心规则：目标点/空地可穿过，箱子/炸弹/墙壁不可穿过）
 */
static uint8 Is_Nav_Passable(const uint8 map[MAP_ROWS][MAP_COLS], int8 y, int8 x) {
    if (x < 0 || x >= MAP_COLS || y < 0 || y >= MAP_ROWS) return 0; // 越界防护
    
    // 目标点(MAP_TARGET)和空地(MAP_EMPTY)均作为畅通路线通行
    if (map[y][x] == MAP_EMPTY || map[y][x] == MAP_TARGET) return 1;
    
    return 0; // 其他所有元素皆为障碍物
}

/**
 * @brief 寻找距离车模最近的箱子的"侧面观察点" (使用BFS曼哈顿扩散)
 * @param map 地图矩阵
 * @param player_pos 车模当前坐标
 * @param target_box_pos (输出参数) 记录最终选定的箱子坐标
 * @return ObservePoint_t 最佳的空地观察坐标
 */
ObservePoint_t Algo_Find_Nearest_Box_Observe_Point(const uint8 map[MAP_ROWS][MAP_COLS], Point_t player_pos, Point_t *target_box_pos) {
    ObservePoint_t result = {{0, 0}, 0};
    int16 head = 0, tail = 0;
    
    memset(visited, 0, sizeof(visited));
    
    // 玩家起点入队
    bfs_queue[tail++] = player_pos;
    visited[player_pos.y][player_pos.x] = 1;
    
    while (head < tail) {
        Point_t current = bfs_queue[head++];
        
        // 1. 检查当前网格四周是否有箱子
        for (int i = 0; i < 4; i++) {
            int8 ny = current.y + dy[i];
            int8 nx = current.x + dx[i];
            
            if (nx >= 0 && nx < MAP_COLS && ny >= 0 && ny < MAP_ROWS) {
                // 如果旁边紧挨着箱子，说明 current 本身就是一个极佳的“观察点”！
                if (map[ny][nx] == MAP_BOX) {
                    result.pos = current;
                    result.is_valid = 1;
                    target_box_pos->x = nx;
                    target_box_pos->y = ny;
                    return result; // 找到最近的观察点，直接中断扩散并返回
                }
            }
        }
        
        // 2. 没看到箱子，继续向四周可通行的区域扩散 (水波纹式蔓延)
        for (int i = 0; i < 4; i++) {
            int8 ny = current.y + dy[i];
            int8 nx = current.x + dx[i];
            
            if (Is_Nav_Passable(map, ny, nx) && !visited[ny][nx]) {
                visited[ny][nx] = 1;
                bfs_queue[tail].x = nx;
                bfs_queue[tail].y = ny;
                tail++;
            }
        }
    }
    return result; // 异常情况：地图封闭，没找到任何可到达的箱子观察点
}

/**
 * @brief 点到点纯寻路算法 (不推箱，专用于侦查阶段的快速跑位)
 * @param map 地图矩阵
 * @param start 起点坐标
 * @param end 终点坐标 (通常是上一个函数求出的 ObservePoint_t.pos)
 * @param result_path 结构体指针，用于接收解算出的路径队列
 * @return uint8 1: 成功找到路径, 0: 无法到达目标点
 */
uint8 Algo_Nav_BFS(const uint8 map[MAP_ROWS][MAP_COLS], Point_t start, Point_t end, NavPath_t *result_path) {
    int16 head = 0, tail = 0;
    uint8 found = 0;
    
    // 1. 数据初始化
    result_path->step_count = 0;
    if (start.x == end.x && start.y == end.y) {
        return 1; // 已经在终点，原地待命
    }
    
    memset(visited, 0, sizeof(visited));
    // 初始父节点矩阵为-1
    for(int i = 0; i < MAP_ROWS; i++) {
        for(int j = 0; j < MAP_COLS; j++) {
            parent_map[i][j].x = -1;
            parent_map[i][j].y = -1;
        }
    }
    
    // 2. 起点入队
    bfs_queue[tail++] = start;
    visited[start.y][start.x] = 1;
    
    // 3. 开始 BFS 寻路探索
    while (head < tail) {
        Point_t current = bfs_queue[head++];
        
        // 踩到终点，停止探索
        if (current.x == end.x && current.y == end.y) {
            found = 1;
            break;
        }
        
        // 尝试向四周合法节点扩展
        for (int i = 0; i < 4; i++) {
            int8 ny = current.y + dy[i];
            int8 nx = current.x + dx[i];
            
            // 必须合法、可通行，且尚未踩过
            if (Is_Nav_Passable(map, ny, nx) && !visited[ny][nx]) {
                visited[ny][nx] = 1;
                parent_map[ny][nx] = current; // 核心：记录地图这个点是从哪儿走过来的
                
                bfs_queue[tail].x = nx;
                bfs_queue[tail].y = ny;
                tail++;
            }
        }
    }
    
    // 4. 路径逆向回溯解析
    if (found) {
        Point_t temp_path[200];
        uint16 step = 0;
        Point_t curr = end;
        
        // 从终点一路倒推回起点 (不将起点自身推入路径队列)
        while (curr.x != start.x || curr.y != start.y) {
            temp_path[step++] = curr;
            curr = parent_map[curr.y][curr.x];
        }
        
        // 倒序装填，使其变成：第一步、第二步...终点
        result_path->step_count = step;
        for (int i = 0; i < step; i++) {
            result_path->path[i] = temp_path[step - 1 - i];
        }
        return 1; // 解算成功
    }
    
    return 0; // 陷入死胡同，无法到达
}