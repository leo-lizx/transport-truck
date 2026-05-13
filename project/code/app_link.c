/*********************************************************************************************************************
 * 文件名称   : app_link.c
 * 模块功能   : 视觉端 (OpenART) 与主控端 (RT1064) 串口帧协议解析层 — 实现文件
 * 任务编号   : P0-1
 *--------------------------------------------------------------------------------------------------------------------
 * 设计要点:
 *   1. 状态机式逐字节解析, 任意非法字节均 fallback 到等 SOF1, 不会卡死
 *   2. CRC8 查表实现 (256 B ROM), 单帧 (194B) 校验 < 50 µs
 *   3. 字节超时: 50 ms 内未收到下一字节 → 状态机复位 (防止半截帧污染下一帧)
 *   4. MAP 帧: 兼容 LEN=192 纯地图与 LEN=194 地图+车辆坐标; 地图字符非法则丢弃
 *   5. 写 g_game_map 用 const 字符 → 枚举的查表映射, 与 app_game_logic.h 中 MAP_* 枚举对齐
 *   6. 不调用任何阻塞 API, 不分配堆内存
 *********************************************************************************************************************/

#include "app_link.h"
#include "app_game_logic.h"
#include "zf_common_typedef.h"
#include "zf_common_headfile.h"     /* P0-3: 间接引入 core_cm7.h, 提供 __DMB() 内存屏障  */

/*===================================================================================================================
 * 分区索引:
 *   § 1. 状态机上下文 (volatile rx_*) + 可观测变量 + ms 时基     [L  25±]
 *   § 2. 【P0-3】地图权威副本 + Seq-Lock 写 (commit_map_frame)     [L  46±]
 *   § 3. 车辆格坐标快照 (commit_car_snapshot / get_car_snapshot)    [L  64±]
 *   § 4. CRC8 查表 + 累加 + ms 读取 API                            [L 178±]
 *   § 5. MAP 帧落地 (ascii_to_map_cell / commit_map_frame)            [L 218±]
 *   § 6. 地图快照读 (app_link_get_map_snapshot, seq-lock retry ×8)  [L 297±]
 *   § 7. 帧分发 (dispatch_frame: MAP / HEARTBEAT)                    [L 328±]
 *   § 8. 解析器复位 / 对外 API (init / isr_feed_byte / tick)         [L 388±]
 *=================================================================================================================*/

/*===================================================================================================================
 * 内部常量 / 静态变量
 *=================================================================================================================*/
#define APP_LINK_BYTE_TIMEOUT_MS    (50U)       /* 字节间超时门槛 (ms)                  */

/*-- 解析器上下文 (volatile: ISR 写, 主循环可观测) ---------------------------------------------------------------*/
static volatile app_link_state_e s_state          = APP_LINK_STATE_WAIT_SOF1;
static volatile uint8            s_rx_type        = 0U;
static volatile uint8            s_rx_len         = 0U;
static volatile uint8            s_rx_idx         = 0U;
static volatile uint8            s_rx_crc_calc    = 0U;     /* 边收边算的 CRC8 累加值 */
static          uint8            s_rx_payload[APP_LINK_MAX_PAYLOAD];

/*===================================================================================================================
 * 全局可观测变量定义
 *=================================================================================================================*/
volatile uint32   g_link_last_map_ms   = 0U;
volatile uint32   g_link_last_car_ms   = 0U;
volatile uint32   g_link_last_hb_ms    = 0U;
volatile uint32   g_link_byte_last_ms  = 0U;
volatile uint8    g_link_car_x         = 0U;
volatile uint8    g_link_car_y         = 0U;
app_link_stats_t  g_link_stats         = {0};

/*-- ms 时基 (由 app_link_tick 累加, 仅 app_link.c 内部使用) ----------------------------------------------------*/
static volatile uint32 s_ms_now = 0U;

/*===================================================================================================================
 * 【P0-3】地图权威副本 + Seq-Lock
 *-------------------------------------------------------------------------------------------------------------------
 * 设计:
 *   - s_map_authoritative: 只由 commit_map_frame() (视觉 UART ISR) 写; 任何读者一律走快照拷贝
 *   - s_map_seq           : 偶 = 稳定, 奇 = 写者正在更新; 写者两次 ++, 读者循环到两次读取相等且偶数为止
 *   - 屏障 __DMB()         : 防止编译器/M7 把数据访问与 seq 自增重排
 *   - 重试上限             : 8 次, 已远超 ISR 单帧写时长 (3 µs) / 读时长 (1 µs) 的可能碰撞窗口
 *=================================================================================================================*/
#define APP_LINK_MAP_SNAPSHOT_RETRY_MAX  (8U)

static uint8           s_map_authoritative[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS] = {{0}};
static volatile uint32 s_map_seq = 0U;     /* 偶 = stable, 奇 = updating                  */

volatile uint32 g_link_map_snapshot_retry_giveup = 0U;

/* 【视觉融合】车辆格坐标权威副本 + seq-lock（仅 ISR 写；读者主循环） */
#define APP_LINK_CAR_SNAPSHOT_RETRY_MAX  (8U)

static volatile uint32 s_car_seq          = 0U;
static volatile uint8  s_car_snap_x       = 0U;
static volatile uint8  s_car_snap_y       = 0U;
static volatile uint32 s_car_snap_ms      = 0U;
static volatile uint32 s_car_frame_id     = 0U;
static volatile uint8  s_car_snap_valid   = 0U;

/*-------------------------------------------------------------------------------------------------------------------
 * ISR 内调用: MAP 载荷已合法写入地图后，再提交车辆坐标（与地图同一帧语义一致）
 *-----------------------------------------------------------------------------------------------------------------*/
static void commit_car_snapshot(uint8 cx, uint8 cy)
{
    s_car_seq++;
    __DMB();
    s_car_snap_x     = cx;
    s_car_snap_y     = cy;
    s_car_snap_ms    = s_ms_now;
    s_car_frame_id++;
    s_car_snap_valid = 1U;
    g_link_car_x     = cx;
    g_link_car_y     = cy;
    g_link_last_car_ms = s_ms_now;
    __DMB();
    s_car_seq++;
}

void app_link_get_car_snapshot(app_link_car_snapshot_t *out)
{
    uint32 retry;
    uint32 s1;
    uint32 s2;

    if (out == NULL) { return; }

    for (retry = 0U; retry <= APP_LINK_CAR_SNAPSHOT_RETRY_MAX; ++retry)
    {
        s1 = s_car_seq;
        if ((s1 & 1U) != 0U) { continue; }
        __DMB();
        out->car_x    = s_car_snap_x;
        out->car_y    = s_car_snap_y;
        out->stamp_ms = s_car_snap_ms;
        out->frame_id = s_car_frame_id;
        out->valid    = s_car_snap_valid;
        __DMB();
        s2 = s_car_seq;
        if (s1 == s2) { return; }
    }

    out->valid = 0U;
}

/*===================================================================================================================
 * CRC8 查表 (多项式 0x07, 初值 0x00, 不反射, 不异或输出 — 即标准 CRC-8/SMBUS 变种)
 *=================================================================================================================*/
static const uint8 k_crc8_table[256] =
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/*-------------------------------------------------------------------------------------------------------------------
 * 工具: 单字节 CRC8 累加
 *-----------------------------------------------------------------------------------------------------------------*/
static inline uint8 crc8_accum(uint8 crc, uint8 byte)
{
    return k_crc8_table[crc ^ byte];
}

uint8 app_link_compute_crc8(const uint8 *data, uint32 len)
{
    uint8 crc = 0x00U;
    uint32 i;
    if (data == NULL) { return 0x00U; }
    for (i = 0U; i < len; ++i)
    {
        crc = k_crc8_table[crc ^ data[i]];
    }
    return crc;
}

uint32 app_link_get_ms(void)
{
    /* 32 位字段读取在 Cortex-M7 上原子, 无需临界区                                  */
    return s_ms_now;
}

/*===================================================================================================================
 * MAP 帧落地: ASCII 字符 → MAP_* 枚举 → g_game_map
 *=================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * 函数: ascii_to_map_cell
 * 功能: 单字符 → 单元格枚举值, 非法字符返回 0xFF
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 ascii_to_map_cell(uint8 ch)
{
    /* 注: 与 algo_sokoban_solver.h 中 MAP_EMPTY/WALL/TARGET/BOX/BOMB 枚举对齐;
     *     '@' (角色) 在视觉端表示当前虚拟车位置, 主控不据此更新 g_game_map
     *     而是依赖里程计自身位姿; 这里把 '@' 当作 EMPTY 处理 (不写入车位).      */
    switch (ch)
    {
        case '-': return (uint8)MAP_EMPTY;
        case '#': return (uint8)MAP_WALL;
        case '.': return (uint8)MAP_TARGET;
        case '$': return (uint8)MAP_BOX;
        case '*': return (uint8)MAP_BOMB;
        case '@': return (uint8)MAP_EMPTY;
        default : return 0xFFU;        /* 非法字符 (含 '?' 越界标记) */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数: commit_map_frame
 * 功能: 把 192 字节 ASCII 载荷写入权威地图副本 s_map_authoritative (CRC 已通过)
 * 返回: 1 成功, 0 因载荷含非法字符而拒绝
 * 备注: 整张地图先写入栈上临时缓冲, 全部合法后再以 seq-lock 提交; 写期间允许被高优先级 ISR 抢占,
 *       读者会自动重试. 不再写 g_game_map (现已降级为主循环私有快照, 由
 *       Game_Logic_Task_Run() 入口 app_link_get_map_snapshot() 刷新).
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 commit_map_frame(const uint8 *payload)
{
    uint8 tmp[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS];
    uint32 r;
    uint32 c;
    uint32 idx = 0U;

    for (r = 0U; r < (uint32)APP_LINK_MAP_ROWS; ++r)
    {
        for (c = 0U; c < (uint32)APP_LINK_MAP_COLS; ++c)
        {
            uint8 cell = ascii_to_map_cell(payload[idx++]);
            if (cell == 0xFFU)
            {
                return 0U;          /* 拒绝整帧, 权威副本保持上一帧 */
            }
            tmp[r][c] = cell;
        }
    }

    /* ---- seq-lock 写: ++seq (奇=updating) -> 拷贝 -> ++seq (偶=stable) ---- */
    s_map_seq++;
    __DMB();
    for (r = 0U; r < (uint32)APP_LINK_MAP_ROWS; ++r)
    {
        for (c = 0U; c < (uint32)APP_LINK_MAP_COLS; ++c)
        {
            s_map_authoritative[r][c] = tmp[r][c];
        }
    }
    __DMB();
    s_map_seq++;

    return 1U;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数: app_link_get_map_snapshot
 * 功能: seq-lock 读, 把 s_map_authoritative 拷贝给调用方 (主循环 / 任意 ISR 均可)
 * 备注:
 *   1. 写者本身只在 LPUART1 ISR 内, 单次写 ~3µs; 读者最多重试 8 次仍碰撞才放弃
 *   2. 放弃时 dst 内容可能含部分上次拷贝/部分新数据; 调用方应忽略本帧 (此处不主动清零, 避免覆盖上次有效快照)
 *-----------------------------------------------------------------------------------------------------------------*/
void app_link_get_map_snapshot(uint8 dst[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS])
{
    uint32 retry;
    uint32 s1;
    uint32 s2;
    uint32 r;
    uint32 c;

    if (dst == NULL) { return; }

    for (retry = 0U; retry <= APP_LINK_MAP_SNAPSHOT_RETRY_MAX; ++retry)
    {
        s1 = s_map_seq;
        if ((s1 & 1U) != 0U)
        {
            continue;                       /* 写者正在更新, 直接重试   */
        }
        __DMB();
        for (r = 0U; r < (uint32)APP_LINK_MAP_ROWS; ++r)
        {
            for (c = 0U; c < (uint32)APP_LINK_MAP_COLS; ++c)
            {
                dst[r][c] = s_map_authoritative[r][c];
            }
        }
        __DMB();
        s2 = s_map_seq;
        if (s1 == s2)
        {
            return;                         /* 拷贝期间无写入, 成功     */
        }
        /* 否则写者插入过, 继续重试 */
    }

    ++g_link_map_snapshot_retry_giveup;     /* 8 次仍未成功, 计数报警   */
}

/*===================================================================================================================
 * 帧分发
 *=================================================================================================================*/
static void dispatch_frame(void)
{
    switch ((app_link_type_e)s_rx_type)
    {
        case APP_LINK_TYPE_MAP:
        {
            uint8 car_ready = 0U;
            uint8 cx        = 0U;
            uint8 cy        = 0U;

            if ((s_rx_len != (uint8)APP_LINK_MAP_PAYLOAD_LEN) &&
                (s_rx_len != (uint8)APP_LINK_MAP_WITH_POS_LEN))
            {
                ++g_link_stats.frames_len_err;
                break;
            }
            if (s_rx_len == (uint8)APP_LINK_MAP_WITH_POS_LEN)
            {
                cx = s_rx_payload[APP_LINK_MAP_PAYLOAD_LEN];
                cy = s_rx_payload[APP_LINK_MAP_PAYLOAD_LEN + 1U];
                if ((cx >= (uint8)APP_LINK_MAP_COLS) || (cy >= (uint8)APP_LINK_MAP_ROWS))
                {
                    ++g_link_stats.frames_len_err;
                    break;
                }
                car_ready = 1U;
            }
            /* 先落地地图，成功后再提交车辆坐标，避免地图被拒而车位已更新的撕裂 */
            if (commit_map_frame((const uint8 *)s_rx_payload) == 0U)
            {
                ++g_link_stats.frames_len_err;
                break;
            }
            if (car_ready != 0U)
            {
                commit_car_snapshot(cx, cy);
            }
            ++g_link_stats.frames_ok;
            g_link_last_map_ms = s_ms_now;
            g_link_last_hb_ms  = s_ms_now;
            break;
        }

        case APP_LINK_TYPE_HEARTBEAT:
        {
            if (s_rx_len != 1U)
            {
                ++g_link_stats.frames_len_err;
                break;
            }
            ++g_link_stats.frames_ok;
            ++g_link_stats.hb_cnt;
            g_link_stats.last_hb_seq = s_rx_payload[0];
            g_link_last_hb_ms = s_ms_now;
            break;
        }

        default:
        {
            ++g_link_stats.frames_unknown_type;
            break;
        }
    }
}

/*===================================================================================================================
 * 解析器复位
 *=================================================================================================================*/
static inline void parser_reset(void)
{
    s_state       = APP_LINK_STATE_WAIT_SOF1;
    s_rx_type     = 0U;
    s_rx_len      = 0U;
    s_rx_idx      = 0U;
    s_rx_crc_calc = 0U;
}

/*===================================================================================================================
 * 对外 API 实现
 *=================================================================================================================*/

void app_link_init(void)
{
    parser_reset();

    g_link_last_map_ms  = 0U;
    g_link_last_car_ms  = 0U;
    g_link_last_hb_ms   = 0U;
    g_link_byte_last_ms = 0U;
    g_link_car_x        = 0U;
    g_link_car_y        = 0U;
    s_ms_now            = 0U;

    /* 车辆坐标快照清零 */
    s_car_seq++;
    __DMB();
    s_car_snap_x       = 0U;
    s_car_snap_y       = 0U;
    s_car_snap_ms      = 0U;
    s_car_frame_id     = 0U;
    s_car_snap_valid   = 0U;
    __DMB();
    s_car_seq++;

    g_link_stats.frames_ok           = 0U;
    g_link_stats.frames_crc_err      = 0U;
    g_link_stats.frames_len_err      = 0U;
    g_link_stats.frames_byte_timeout = 0U;
    g_link_stats.frames_unknown_type = 0U;
    g_link_stats.sync_drops          = 0U;
    g_link_stats.last_hb_seq         = 0U;
    g_link_stats.hb_cnt              = 0U;

    /* P0-3: seq-lock 计数与权威副本归零 */
    g_link_map_snapshot_retry_giveup = 0U;
    /* s_map_seq 保持累计 (即便 init 重入也不破坏奇偶语义); 副本归零保证读到 MAP_EMPTY */
    {
        uint32 r;
        uint32 c;
        s_map_seq++;
        __DMB();
        for (r = 0U; r < (uint32)APP_LINK_MAP_ROWS; ++r)
        {
            for (c = 0U; c < (uint32)APP_LINK_MAP_COLS; ++c)
            {
                s_map_authoritative[r][c] = (uint8)MAP_EMPTY;
            }
        }
        __DMB();
        s_map_seq++;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * ISR 入口: 单字节状态机
 *-----------------------------------------------------------------------------------------------------------------*/
void app_link_isr_feed_byte(uint8 byte)
{
    g_link_byte_last_ms = s_ms_now;

    switch (s_state)
    {
        case APP_LINK_STATE_WAIT_SOF1:
        {
            if (byte == APP_LINK_SOF1)
            {
                s_state = APP_LINK_STATE_WAIT_SOF2;
            }
            else
            {
                ++g_link_stats.sync_drops;          /* 同步前的散字节, 计数即可 */
            }
            break;
        }

        case APP_LINK_STATE_WAIT_SOF2:
        {
            if (byte == APP_LINK_SOF2)
            {
                s_state = APP_LINK_STATE_WAIT_TYPE;
            }
            else if (byte == APP_LINK_SOF1)
            {
                /* 连续 0xAA: 当前字节可能就是新一帧的 SOF1, 留在 WAIT_SOF2 */
            }
            else
            {
                ++g_link_stats.sync_drops;
                parser_reset();
            }
            break;
        }

        case APP_LINK_STATE_WAIT_TYPE:
        {
            s_rx_type     = byte;
            s_rx_crc_calc = crc8_accum(0x00U, byte);
            s_state       = APP_LINK_STATE_WAIT_LEN;
            break;
        }

        case APP_LINK_STATE_WAIT_LEN:
        {
            if (byte > (uint8)APP_LINK_MAX_PAYLOAD)
            {
                /* LEN 越界: 视为错帧, 复位重新等同步                                     */
                ++g_link_stats.frames_len_err;
                parser_reset();
                break;
            }
            s_rx_len      = byte;
            s_rx_crc_calc = crc8_accum(s_rx_crc_calc, byte);
            s_rx_idx      = 0U;
            s_state       = (s_rx_len == 0U)
                            ? APP_LINK_STATE_WAIT_CRC
                            : APP_LINK_STATE_WAIT_PAYLOAD;
            break;
        }

        case APP_LINK_STATE_WAIT_PAYLOAD:
        {
            s_rx_payload[s_rx_idx++] = byte;
            s_rx_crc_calc = crc8_accum(s_rx_crc_calc, byte);
            if (s_rx_idx >= s_rx_len)
            {
                s_state = APP_LINK_STATE_WAIT_CRC;
            }
            break;
        }

        case APP_LINK_STATE_WAIT_CRC:
        {
            if (byte == s_rx_crc_calc)
            {
                dispatch_frame();
            }
            else
            {
                ++g_link_stats.frames_crc_err;
            }
            parser_reset();
            break;
        }

        default:
        {
            parser_reset();
            break;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 时基钩子: 累加 ms 计数 + 字节超时检测
 *-----------------------------------------------------------------------------------------------------------------*/
void app_link_tick(uint32 elapsed_ms)
{
    s_ms_now += elapsed_ms;

    /* 字节超时: 状态机已离开等同步, 但长时间没下一字节 → 复位                          */
    if (s_state != APP_LINK_STATE_WAIT_SOF1)
    {
        if ((s_ms_now - g_link_byte_last_ms) > APP_LINK_BYTE_TIMEOUT_MS)
        {
            ++g_link_stats.frames_byte_timeout;
            parser_reset();
        }
    }
}
