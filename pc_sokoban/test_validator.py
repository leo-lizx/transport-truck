"""快速单元测试"""
import sys
sys.path.insert(0, '.')

try:
    import sokoban_validator as sv
    print("导入成功")
except Exception as e:
    print(f"导入失败: {e}")
    import traceback; traceback.print_exc()
    sys.exit(1)

# 测试1: 单箱BFS
print("\n=== 测试1: 单箱BFS ===")
m = sv._make_empty_map()
m[3][3] = sv.BOX
m[3][10] = sv.TARGET
sol = sv.sokoban_bfs_single(m, (5, 2), (3, 3), (3, 10))
assert sol is not None, "BFS 应有解"
print(f"步数: {len(sol)}")
d = sv.actions_to_string(sol, (5, 2), (3, 3), use_unicode=False)
print(f"方向串: {d}")
valid, msg = sv.validate_path(m, sol, (5, 2), (3, 3))
assert valid, f"路径校验失败: {msg}"
print(f"路径校验: {msg}")

# 测试2: Stage1 时间优化
print("\n=== 测试2: Stage1 时间优化（2箱） ===")
m2 = sv._make_empty_map()
m2[2][2] = sv.BOX
m2[2][12] = sv.BOX
m2[8][2] = sv.TARGET
m2[8][12] = sv.TARGET
r = sv.solve_stage1(m2, (5, 7))
assert r is not None, "Stage1 应有解"
print(f"总步数: {r['total_steps']}")

# 测试2a: Stage1 公开入口按“执行 + 返库”时间成本优于原贪心。
print("\n=== 测试2a: Stage1 连续计时时间优化 ===")
m2a, p2a, err2a = sv.parse_map_text("""
################
#----------.---#
#--------------#
#--------#-#---#
#-----#-##-----#
#----#--$------#
#--##--#---.---#
#--------------#
#----##--###---#
#------$-------#
#--------@-----#
################
""")
assert err2a == "", err2a
greedy2a = sv._solve_stage1_greedy(m2a, p2a)
stage1_2a = sv.solve_stage1(m2a, p2a)
assert greedy2a is not None
assert stage1_2a is not None

greedy_cost2a = 0
cur2a = p2a
for sub2a in greedy2a['sub_solutions']:
    flags2a = sv._push_flags(sub2a['actions'], cur2a,
                             greedy2a['boxes'][sub2a['box_idx']])
    greedy_cost2a += sv.sequence_time_cost(sub2a['actions'], flags2a)
    cur2a = sub2a['player_end']
greedy_return2a = sv.build_return_path(m2a, cur2a, p2a)
assert greedy_return2a is not None
greedy_cost2a += greedy_return2a['time_cost_ms']
assert stage1_2a['time_cost_ms'] < greedy_cost2a
print(f"原贪心 {greedy_cost2a}ms → 时间优化 {stage1_2a['time_cost_ms']}ms")

# 测试3: 地图生成
print("\n=== 测试3: 地图生成 ===")
for seed in [1, 2, 42]:
    m3, p3 = sv.generate_map(stage=1, box_count=2, seed=seed)
    r3 = sv.solve_level(1, m3, p3)
    status = f"OK steps={r3['total_steps']}" if r3 else "UNSOLVABLE"
    print(f"  seed={seed}: {status}")

# 测试4: 侦查阶段
print("\n=== 测试4: Stage2 侦查 ===")
m4, p4 = sv.generate_map(stage=2, box_count=2, seed=42)
scout = sv.plan_scout_phase(m4, p4)
assert scout['all_visited'], "应访问全部箱子"
sol4 = sv.solve_level(2, m4, p4)
assert sol4 and sol4['sub_solutions'][0]['phase'] == 'scout'
print(f"侦查步数: {len(scout['scout_actions'])}  总步数: {sol4['total_steps']}")

# 测试5: Stage3 炸弹
print("\n=== 测试5: Stage3 炸弹 ===")
m5, p5 = sv.generate_map(stage=3, box_count=2, seed=7)
sol5 = sv.solve_level(3, m5, p5)
assert sol5 is not None, "Stage3 应有解"
print(f"总步数: {sol5['total_steps']}")

# 测试6: 地图导入
print("\n=== 测试6: 地图导入 ===")
text = sv.export_map_text(m2, (5, 7))
m6, p6, err = sv.parse_map_text(text)
assert err == "" and p6 == (5, 7)
print("往返导入 OK")

# 测试7: 指令验证
print("\n=== 测试7: 指令验证 ===")
script = sv.build_script_from_solution(r)
vr = sv.verify_run_script(m2, (5, 7), script, stage=1)
assert vr['ok'], vr['message']
print(vr['message'])

# 测试8: V2 侦查 — 两类物体统一规划，全部实地观察（排除法推断已移除）
print("\n=== 测试8: V2 侦查 (混合 Tour + 全部实地观察) ===")
m8, p8 = sv.generate_map(stage=2, box_count=3, seed=11)
scout8 = sv.plan_scout_phase_v2(m8, p8)
total_items = 6  # 3 box + 3 target
print(f"  实地访问: {scout8['visited_count']} / {total_items}")
assert scout8['visited_count'] == total_items
assert scout8['inferred_count'] == 0
assert scout8['all_visited'], "应全部 resolved"
assert scout8['box_to_target_idx'] is not None
kinds8 = [v['kind'] for v in scout8['visits']]
assert kinds8.count('box') == 3 and kinds8.count('target') == 3
assert any(kind == 'box' for kind in kinds8[kinds8.index('target') + 1:]), kinds8
for v in scout8['visits']:
    fd = v.get('face_dir')
    assert fd is not None, "每个实地观察项都应有面向物体的 face_dir"
    obs = v['observe']
    pos = v['pos']
    assert (obs[0] + sv.DR[fd], obs[1] + sv.DC[fd]) == pos, "face_dir 应从观察点指向物体"
print(f"  box_classes={scout8['box_classes']} target_classes={scout8['target_classes']}")
print(f"  box→target 映射: {scout8['box_to_target_idx']}")

# 测试8a: 小规模侦查把箱子和目标放进同一个全局 tour
print("\n=== 测试8a: V2 混合全局侦查顺序 ===")
m8a, p8a = sv.generate_map(stage=2, box_count=2, seed=4)
scout8a = sv.plan_scout_phase_v2(m8a, p8a)
order8a = [(v['kind'], v['item_idx']) for v in scout8a['visits']]
assert order8a == [('target', 1), ('target', 0), ('box', 0), ('box', 1)], order8a
assert len(scout8a['scout_actions']) == 12, len(scout8a['scout_actions'])
assert scout8a['inferred_count'] == 0
print(f"  顺序: {order8a}  侦查步数: {len(scout8a['scout_actions'])}")

# 测试8b: 同图案多箱/多目标时按组内最少静态推数匹配
print("\n=== 测试8b: 重复图案最少推数匹配 ===")
m8b = sv._make_empty_map()
m8b[2][2] = sv.BOX
m8b[8][10] = sv.BOX
m8b[2][10] = sv.TARGET
m8b[8][2] = sv.TARGET
scout8b = sv.plan_scout_phase_v2(
    m8b, (5, 6),
    box_classes=[1, 1],
    target_classes=[1, 1],
)
assert scout8b['all_visited']
assert scout8b['inferred_count'] == 0
assert scout8b['box_to_target_idx'] == [1, 0], scout8b['box_to_target_idx']
print(f"  重复 class=1 映射: {scout8b['box_to_target_idx']} (期望 [1, 0])")

# Test 8c: 重复类别不能只按曼哈顿距离配对。贴上边界的箱子无法向下起推，
# 旧映射会选 [1,0] 并判无解；静态反向推箱距离应改选可解的 [0,1]。
print("\n=== 测试8c: 重复类别静态推送可行性 ===")
m8c, p8c, err8c = sv.parse_map_text("""
################
#-$-----.------#
#-.------------#
#--------------#
#--------------#
#--------------#
#-----$--#-----#
#--------------#
#@-------#-----#
#--------------#
#---#-----#----#
################
""")
assert err8c == "", err8c
scout8c = sv.plan_scout_phase_v2(m8c, p8c, [1, 1], [1, 1])
assert scout8c['all_visited']
assert scout8c['inferred_count'] == 0
assert scout8c['box_to_target_idx'] == [0, 1], scout8c['box_to_target_idx']
assert sv.solve_stage2(m8c, scout8c['player_after_scout'],
                       scout8c['box_to_target_idx']) is not None
print("静态不可推的近目标被排除，同类别映射选择可解组合 [0, 1]。")

# Test 8d: the exact tour must execute its chosen observe cell and compare
# movement segments together with the final observation turn.
print("\n=== Test 8d: time-aware scout observe selection ===")
m8d = sv._make_empty_map()
m8d[5][5] = sv.TARGET
m8d[8][12] = sv.BOX
scout8d = sv.plan_scout_phase_v2(
    m8d, (5, 2), box_classes=[1], target_classes=[1])
assert scout8d['all_visited']
assert scout8d['visits'][0]['observe'] == (5, 4), scout8d['visits'][0]
assert scout8d['visits'][0]['face_dir'] == 3, scout8d['visits'][0]
print("精确 Tour 保留观察格，并按平移停站与观察转角的总时间选点。")

# Test 8e: maps above the six-item exact-DP limit use the same turn-aware
# observe scoring in the greedy fallback.
print("\n=== Test 8e: turn-aware greedy scout fallback ===")
m8e = sv._make_empty_map()
for r, c in [(5, 5), (1, 10), (2, 12), (4, 12),
             (6, 12), (8, 12), (9, 10)]:
    m8e[r][c] = sv.BOX
for r, c in [(1, 2), (2, 4), (3, 6), (7, 4),
             (8, 6), (9, 6), (9, 8)]:
    m8e[r][c] = sv.TARGET
scout8e = sv.plan_scout_phase_v2(
    m8e, (5, 2), box_classes=[1, 2, 3, 4, 5, 6, 7],
    target_classes=[1, 2, 3, 4, 5, 6, 7])
assert scout8e['all_visited']
assert scout8e['visited_count'] == 14
assert scout8e['inferred_count'] == 0
assert scout8e['visits'][0]['pos'] == (5, 5), scout8e['visits'][0]
assert scout8e['visits'][0]['observe'] == (5, 4), scout8e['visits'][0]
print("超过精确 DP 上限时，两步前瞻仍使用相同的时间代价。")

print("\n=== Test 8f: duplicate classes are fully observed on site ===")
m8f_infer = sv._make_empty_map()
for pos in [(2, 2), (5, 5), (8, 8)]:
    m8f_infer[pos[0]][pos[1]] = sv.BOX
for pos in [(2, 10), (5, 10), (8, 10)]:
    m8f_infer[pos[0]][pos[1]] = sv.TARGET
scout8f_infer = sv.plan_scout_phase_v2(
    m8f_infer, (9, 2), box_classes=[1, 1, 3],
    target_classes=[3, 1, 1])
assert scout8f_infer['all_visited']
assert scout8f_infer['visited_count'] == 6
assert scout8f_infer['inferred_count'] == 0
assert sorted(scout8f_infer['box_classes']) == sorted(scout8f_infer['target_classes'])
print(f"重复类别全部实地确认: target={scout8f_infer['target_classes']}")

print("\n=== Test 8g: direction-state navigation avoids extra stops ===")
m8f = sv._make_empty_map()
m8f[5][4] = sv.WALL
route8f = sv.nav_time_path(m8f, (5, 2), (5, 6))
assert route8f is not None
actions8f = sv._path_to_actions(route8f[0])
segments8f = 1 + sum(a != b for a, b in zip(actions8f, actions8f[1:]))
assert route8f[1] == len(actions8f) + segments8f * sv.NAV_TIME_TURN_UNITS
assert segments8f == 3, (route8f, actions8f)
print(f"方向状态路径: {len(actions8f)} 格 / {segments8f} 段 / 代价 {route8f[1]}")

print("\n=== Test 8h: unresolved mixed-tour item blocks completion ===")
m8h = sv._make_empty_map()
m8h[3][3] = sv.BOX
m8h[7][3] = sv.BOX
m8h[3][10] = sv.TARGET
m8h[7][10] = sv.TARGET
scout8h = sv.plan_scout_phase_v2(
    m8h, (5, 2), box_classes=[0, 2], target_classes=[1, 2])
assert not scout8h['all_visited']
assert scout8h['inferred_count'] == 0
assert 0 in scout8h['box_classes']
print("混合 Tour 可先确认其他物体，但无有效类别的箱子仍会阻止最终配对。")

print("\n=== Test 8i: target class outside remaining box multiset is rejected ===")
scout8i = sv.plan_scout_phase_v2(
    m8h, (5, 2), box_classes=[1, 2], target_classes=[9, 9])
assert not scout8i['all_visited']
assert scout8i['inferred_count'] == 0
assert scout8i['target_classes'].count(9) <= scout8i['box_classes'].count(9) + \
       scout8i['box_classes'].count(0)
print("混合 Tour 仅在剩余异类槽位仍可能配平时接受当前类别。")

# 测试9: V2 + Stage2 完整求解 (走真实路径)
print("\n=== 测试9: V2 完整求解 ===")
sol9 = sv.solve_level(2, m8, p8)
assert sol9 is not None and sol9['scout']['all_visited']
print(f"  总步数: {sol9['total_steps']}  侦查访问: {sol9['scout']['visited_count']}")

# 测试10: Stage1 选箱使用实际 BFS 距离而非曼哈顿距离
print("\n=== 测试10: Stage1 BFS 选箱距离 ===")
m10 = sv._make_empty_map()
for row in range(1, 9):
    m10[row][4] = sv.WALL
m10[2][5] = sv.BOX
m10[7][2] = sv.BOX
m10[9][2] = sv.TARGET
m10[9][5] = sv.TARGET
greedy10 = sv._solve_stage1_greedy(m10, (2, 2))
assert greedy10 is not None
assert greedy10['sub_solutions'][0]['box_idx'] == 1
print("墙体绕障场景下按实际 BFS 距离选择箱子")

# 测试10b: Stage2 固定映射也应按实际 BFS 距离选箱，而不是退回曼哈顿距离。
print("\n=== 测试10b: Stage2 BFS 选箱距离 ===")
mapping10b = [1, 0]
greedy10b = sv._solve_stage2_greedy(m10, (2, 2), mapping10b)
assert greedy10b is not None
assert greedy10b['sub_solutions'][0]['box_idx'] == 1
assert greedy10b['sub_solutions'][0]['target_idx'] == 0
print("固定映射场景下同样优先选择实际可近的箱子")

# Tests 10c/10d: the nearest pair/box can be locally unsolvable while a later
# candidate gives a complete solution. The feasible-greedy fallback must keep
# searching instead of returning a false negative on the first failed BFS.
print("\n=== Test 10c: Stage1 feasible-candidate fallback ===")
m10c, p10c, err10c = sv.parse_map_text("""
################
#--------------#
#--------------#
#-----.--------#
#---##-##------#
#---@$$-.------#
#---##-##------#
#--------------#
#--------------#
#--------------#
#--------------#
################
""")
assert err10c == "", err10c
greedy10c = sv._solve_stage1_greedy(m10c, p10c)
assert greedy10c is not None
assert [(s['box_idx'], s['target_idx']) for s in greedy10c['sub_solutions']] == [(1, 0), (0, 1)]
print("Stage1 skips an infeasible nearest pairing and completes all boxes.")

print("\n=== Test 10d: Stage2 feasible-box fallback ===")
m10d, p10d, err10d = sv.parse_map_text("""
################
#--------------#
#--------------#
#-----.--------#
#---##-##------#
#---@$$-.------#
#---##-##------#
#--------------#
#--------------#
#--------------#
#--------------#
################
""")
assert err10d == "", err10d
greedy10d = sv._solve_stage2_greedy(m10d, p10d, [1, 0])
assert greedy10d is not None
assert [s['box_idx'] for s in greedy10d['sub_solutions']] == [1, 0]
print("Stage2 skips an infeasible nearest box and preserves the fixed mapping.")

# Test 11: 赛规实测——箱子推到不对应目标点只是不消去，可以被推着穿过。
# 走廊图中把箱子推到远端目标必须穿过近端未用目标，现应可解。
print("\n=== Test 11: box may pass through an unused target ===")
m11 = sv._make_empty_map()
for c in range(2, 9):
    m11[4][c] = sv.WALL
    m11[6][c] = sv.WALL
m11[5][3] = sv.BOX
m11[5][5] = sv.TARGET
m11[5][8] = sv.TARGET
boxes11 = sv.extract_elements(m11, sv.BOX)
targets11 = sv.extract_elements(m11, sv.TARGET)
sub11_far = sv.build_sub_map(
    m11, boxes11, targets11,
    [False], [False, False],
    0, 1,
)
sol11_far = sv.sokoban_bfs_single(sub11_far, (5, 2), boxes11[0], targets11[1])
assert sol11_far is not None, "穿过未用目标推到远端目标应可解"
end11 = sv.simulate_push_actions(sub11_far, (5, 2), boxes11[0], sol11_far) \
    if hasattr(sv, 'simulate_push_actions') else None
sub11_near = sv.build_sub_map(
    m11, boxes11, targets11,
    [False], [False, False],
    0, 0,
)
assert sv.sokoban_bfs_single(sub11_near, (5, 2), boxes11[0], targets11[0]) is not None
print("Box passes through the unused target; both corridor targets are solvable.")

# Test 12: current six-box first-level regression. The objective includes the
# direct return to the formal garage and must beat the old feasible greedy plan.
print("\n=== Test 12: Stage1 six-box continuous-time regression ===")
m12, _, err12 = sv.parse_map_text("""
################
#-----.--------#
#-#--#--------.#
#-$--#-#---###-#
#--$##-----.$--#
#----------.-#-#
#@---##-#-----$#
#---##-$--#--#-#
#--------#---#-#
#------.#-#-#--#
#----$------.--#
################
""")
assert err12 == "", err12
p12 = (5, 1)
greedy12 = sv._solve_stage1_greedy(m12, p12)
opt12 = sv.solve_stage1(m12, p12, home_pos=(5, 1))
assert greedy12 is not None and opt12 is not None
greedy_cost12 = 0
cur12 = p12
for sub12 in greedy12['sub_solutions']:
    flags12 = sv._push_flags(sub12['actions'], cur12,
                             greedy12['boxes'][sub12['box_idx']])
    greedy_cost12 += sv.sequence_time_cost(sub12['actions'], flags12)
    cur12 = sub12['player_end']
ret12 = sv.build_return_path(m12, cur12, (5, 1))
assert ret12 is not None
greedy_cost12 += ret12['time_cost_ms']
assert opt12['total_steps'] <= 85
assert opt12['time_cost_ms'] < greedy_cost12
assert opt12['return_plan']['direct']
assert opt12['return_plan']['waypoints'] == [((5, 1), 'critical')]
print(f"greedy={greedy12['total_steps']}步/{greedy_cost12}ms, "
      f"optimized={opt12['total_steps']}步/{opt12['time_cost_ms']}ms")

print("\n=== Test 12b: Stage1 pair-budget fallback ===")
old_pair_limit12b = sv.STAGE1_PAIR_LIMIT
try:
    sv.STAGE1_PAIR_LIMIT = 1
    fallback12b = sv.solve_stage1(m12, p12, home_pos=(5, 1))
finally:
    sv.STAGE1_PAIR_LIMIT = old_pair_limit12b
assert fallback12b is not None
assert fallback12b['pair_evaluations'] == 1
assert fallback12b['return_plan']['direct']
print("预算耗尽且尚无完整叶子时，保留贪心可行解而不误报死局。")

print("\n=== Test 13: typed waypoints and direct return ===")
typed13 = sv.actions_to_typed_waypoints(
    [sv.DIR_LETTER.index('R'), sv.DIR_LETTER.index('R'),
     sv.DIR_LETTER.index('D'), sv.DIR_LETTER.index('D')],
    [False, True, False, False],
    (5, 1),
)
assert typed13 == [((5, 3), 'critical'), ((7, 3), 'walk')]
m13 = sv._make_empty_map()
m13[5][2] = sv.WALL
ret13 = sv.build_return_path(m13, (5, 3), (5, 1))
assert ret13 is not None and ret13['direct']
assert ret13['actions'] == []
assert ret13['waypoints'] == [((5, 1), 'critical')]
assert ret13['time_cost_ms'] == 600
print("普通转弯/关键推箱航点标记正确，返库命令忽略虚拟墙且不计已关闭的 Snap。")
print("\n=============================")
print("所有测试通过！")
