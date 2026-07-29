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

print("\n=== Test 8i: mismatched classes finish observation and request rescan ===")
scout8i = sv.plan_scout_phase_v2(
    m8h, (5, 2), box_classes=[1, 2], target_classes=[9, 9])
assert scout8i['all_visited']
assert scout8i['inferred_count'] == 0
assert scout8i['matched_count'] == 0
assert scout8i['box_to_target_idx'] == [
    sv.UNMATCHED_TARGET, sv.UNMATCHED_TARGET]
batch8i = sv.solve_recognized_batch(
    2, m8h, scout8i['player_after_scout'],
    scout8i['box_to_target_idx'])
assert batch8i is not None
assert batch8i['status'] == 'rescan_without_push'
assert batch8i['needs_rescan']
print("错误类别仍完成整轮观察；无匹配项时原地等待新地图重识别。")

print("\n=== Test 8j: partial matches are pushed before rescan ===")
scout8j = sv.plan_scout_phase_v2(
    m8h, (5, 2), box_classes=[1, 2], target_classes=[1, 9])
assert scout8j['all_visited']
assert scout8j['matched_count'] == 1
assert scout8j['box_to_target_idx'] == [0, sv.UNMATCHED_TARGET]
problem8j = sv.build_partial_stage2_problem(
    m8h, scout8j['box_to_target_idx'])
assert problem8j is not None
assert problem8j['map'][7][3] == sv.WALL
assert problem8j['map'][7][10] == sv.EMPTY
batch8j = sv.solve_recognized_batch(
    2, m8h, scout8j['player_after_scout'],
    scout8j['box_to_target_idx'])
assert batch8j is not None and batch8j['status'] == 'pushed'
assert batch8j['matched_count'] == 1 and batch8j['needs_rescan']
assert len(batch8j['push_result']['sub_solutions']) == 1
assert problem8j['map'][batch8j['player_after'][0]][batch8j['player_after'][1]] \
       in (sv.EMPTY, sv.TARGET)
print("未匹配箱保留为障碍；已匹配箱完成后车辆停在空地并进入重读图。")

print("\n=== Test 8k: Stage3 partial batch preserves bomb replanning ===")
m8k = sv._make_empty_map()
for row8k in range(1, 11):
    m8k[row8k][7] = sv.WALL
m8k[4][3] = sv.BOX
m8k[8][3] = sv.BOX
m8k[4][10] = sv.TARGET
m8k[8][10] = sv.TARGET
m8k[5][5] = sv.BOMB
batch8k = sv.solve_recognized_batch(
    3, m8k, (6, 2), [0, sv.UNMATCHED_TARGET])
assert batch8k is not None and batch8k['status'] == 'pushed'
assert batch8k['needs_rescan'] and batch8k['matched_count'] == 1
assert any(phase['kind'] == 'bomb' for phase in batch8k['phases'])
assert batch8k['phases'][-1]['kind'] == 'push'
print("第三关先按已匹配目标炸墙并推箱，随后同样进入剩余地图重识别。")

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

# Test 11b/11c: 第二关目标连续共行/共列时，不识别图案；只用开局地图
# 一次性规划所有箱子，然后连续扫过整条目标线，中间不重读地图。
print("\n=== Test 11b: Stage2 horizontal target-line sweep bypass ===")
m11b = sv._make_empty_map()
for c11b in range(7, 11):
    m11b[5][c11b] = sv.TARGET
for r11b in (2, 4, 6, 8):
    m11b[r11b][2] = sv.BOX
line11b = sv.detect_stage2_line_targets(m11b)
assert line11b == {'horizontal': True, 'fixed': 5}
sol11b = sv.solve_level(
    2, m11b, (5, 2),
    box_classes=[1, 2, 3, 4],
    target_classes=[4, 2, 1, 3],
)
assert sol11b is not None
assert sol11b['scout']['line_sweep_bypass']
assert sol11b['scout']['planning_map_reads'] == 1
assert sol11b['scout']['execution_batches'] == 1
sweeps11b = sol11b['sub_solutions']
assert [sub['phase'] for sub in sweeps11b] == ['line_sweep'] * 4
assert [len(sub['sweep_targets']) for sub in sweeps11b] == [4] * 4
assert all(sub['planned_from_initial_snapshot'] for sub in sweeps11b)
assert {sub['matched_target'] for sub in sweeps11b} == {
    (5, 7), (5, 8), (5, 9), (5, 10)}
for idx11b, sub11b in enumerate(sweeps11b):
    trace11b = sv._planned_box_trace(
        sub11b['actions'], sub11b['push_flags'],
        (5, 2) if idx11b == 0 else sweeps11b[idx11b - 1]['player_end'],
        sub11b['box_start'])
    assert set(sub11b['sweep_targets']).issubset(set(trace11b))
print("四只箱子共用一张开局地图完成规划，一批连续执行且每箱均扫过 4 个目标。")

print("\n=== Test 11c: vertical line detection and conservative trigger ===")
m11c = sv._make_empty_map()
for r11c in range(3, 7):
    m11c[r11c][8] = sv.TARGET
for c11c in (3, 5, 7, 11):
    m11c[9][c11c] = sv.BOX
line11c = sv.detect_stage2_line_targets(m11c)
assert line11c == {'horizontal': False, 'fixed': 8}
sol11c = sv.solve_stage2_line_sweep(
    m11c, (9, 8), line11c,
    box_classes=[1, 2, 3, 4],
    target_classes=[3, 1, 4, 2],
)
assert sol11c is not None and len(sol11c['sub_solutions']) == 4
assert sol11c['planning_map_reads'] == 1
assert sol11c['execution_batches'] == 1
assert all(len(sub['sweep_targets']) == 4
           for sub in sol11c['sub_solutions'])
m11c_gap = sv._make_empty_map()
for c11c in (5, 6, 8, 9):
    m11c_gap[5][c11c] = sv.TARGET
assert sv.detect_stage2_line_targets(m11c_gap) is None
print("竖排同样进入盲扫；带间隔的偶然共线布局仍保留原识别流程。")

print("\n=== Test 11d: one-shot line-sweep planning fallback ===")
m11d = [row[:] for row in m11b]
m11d[5][6] = sv.WALL
m11d[5][11] = sv.WALL
line11d = sv.detect_stage2_line_targets(m11d)
assert line11d is not None
assert sv.solve_stage2_line_sweep(
    m11d, (5, 2), line11d,
    box_classes=[1, 2, 3, 4],
    target_classes=[4, 2, 1, 3],
) is None
sol11d = sv.solve_level(
    2, m11d, (5, 2),
    box_classes=[1, 2, 3, 4],
    target_classes=[4, 2, 1, 3],
)
assert sol11d is not None
assert not sol11d['scout'].get('line_sweep_bypass', False)
print("两端入口都不可用时，全量盲扫不执行任何前缀，整体回退到常规识别。")

print("\n=== Test 11e: directional staging keeps the map unchanged ===")
m11e = sv._make_empty_map()
for c11e in range(7, 11):
    m11e[5][c11e] = sv.TARGET
m11e[4][2] = sv.BOX
m11e[5][6] = sv.WALL
original11e = [row[:] for row in m11e]
problem11e = sv.build_stage2_line_sweep_problem(
    m11e, (5, 2), sv.detect_stage2_line_targets(m11e),
    0, positive_direction=False)
assert problem11e is not None
assert m11e == original11e and problem11e['map'] == original11e
assert problem11e['map'] is m11e
actions11e = sv.sokoban_bfs_single(
    problem11e['map'], (5, 2), problem11e['box_pos'],
    problem11e['staging'], problem11e['sweep_action'])
assert actions11e is not None
flags11e = sv._push_flags(actions11e, (5, 2), problem11e['box_pos'])
trace11e = sv._planned_box_trace(
    actions11e, flags11e, (5, 2), problem11e['box_pos'])
assert [action for action, pushed in zip(actions11e, flags11e)
        if pushed][-1] == sv.DIR_LETTER.index('L')
assert all(row in (4, 5) for row, _ in trace11e)
print("方向终态替代入口虚拟墙；真实地图逐格不变且箱子不再反向绕行。")

print("\n=== Test 11f: staged/aligned boxes enter the sweep directly ===")
m11f = sv._make_empty_map()
for r11f in range(5, 9):
    m11f[r11f][7] = sv.TARGET
for box11f in ((3, 9), (4, 4), (4, 7), (7, 5)):
    m11f[box11f[0]][box11f[1]] = sv.BOX
sol11f = sv.solve_stage2_line_sweep(
    m11f, (5, 2), sv.detect_stage2_line_targets(m11f),
    box_classes=[1, 2, 2, 2],
    target_classes=[1, 2, 2, 2])
assert sol11f is not None
assert sol11f['sub_solutions'][0]['box_start'] == (4, 7)
assert sol11f['sub_solutions'][0]['direct_staging']
assert not any(sol11f['sub_solutions'][0]['push_flags'][:-4])
assert {sub11f['matched_target'] for sub11f in sol11f['sub_solutions']} == {
    (5, 7), (6, 7), (7, 7), (8, 7)}
print("入口已有箱子时车辆只走到箱后；同轴箱子沿目标列直接推入并完成重复号码盲扫。")

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
