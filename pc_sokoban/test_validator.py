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

# 测试2: Stage1 贪心
print("\n=== 测试2: Stage1 贪心（2箱） ===")
m2 = sv._make_empty_map()
m2[2][2] = sv.BOX
m2[2][12] = sv.BOX
m2[8][2] = sv.TARGET
m2[8][12] = sv.TARGET
r = sv.solve_stage1(m2, (5, 7))
assert r is not None, "Stage1 应有解"
print(f"总步数: {r['total_steps']}")

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

# 测试8: V2 侦查 — 必须实地访问全部箱子/目标
print("\n=== 测试8: V2 侦查 (全量实地访问) ===")
m8, p8 = sv.generate_map(stage=2, box_count=3, seed=11)
scout8 = sv.plan_scout_phase_v2(m8, p8)
total_items = 6  # 3 box + 3 target
print(f"  访问数: {scout8['visited_count']}/{total_items} (期望 6/6)")
assert scout8['visited_count'] == total_items, "应实地访问全部物体"
assert scout8['all_visited'], "应全部 resolved"
assert scout8['box_to_target_idx'] is not None
for v in scout8['visits']:
    fd = v.get('face_dir')
    assert fd is not None, "每个实地观察项都应有面向物体的 face_dir"
    obs = v['observe']
    pos = v['pos']
    assert (obs[0] + sv.DR[fd], obs[1] + sv.DC[fd]) == pos, "face_dir 应从观察点指向物体"
print(f"  box_classes={scout8['box_classes']} target_classes={scout8['target_classes']}")
print(f"  box→target 映射: {scout8['box_to_target_idx']}")

# 测试8a: 小规模侦查应使用全局 tour，而不是当前最近点贪心
print("\n=== 测试8a: V2 侦查全局顺序 ===")
m8a, p8a = sv.generate_map(stage=2, box_count=2, seed=4)
scout8a = sv.plan_scout_phase_v2(m8a, p8a)
order8a = [(v['kind'], v['item_idx']) for v in scout8a['visits']]
assert order8a == [('target', 1), ('target', 0), ('box', 0), ('box', 1)], order8a
assert len(scout8a['scout_actions']) == 12, len(scout8a['scout_actions'])
print(f"  顺序: {order8a}  侦查步数: {len(scout8a['scout_actions'])}")

# 测试8b: 同图案多箱/多目标时按组内最短距离匹配
print("\n=== 测试8b: 重复图案最短匹配 ===")
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
assert scout8b['box_to_target_idx'] == [1, 0], scout8b['box_to_target_idx']
print(f"  重复 class=1 映射: {scout8b['box_to_target_idx']} (期望 [1, 0])")

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
# Test 11: a box may enter only the current target, not pass through another
# unused target on the way to a farther target.
print("\n=== Test 11: block pass-through targets ===")
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
assert sv.sokoban_bfs_single(sub11_far, (5, 2), boxes11[0], targets11[1]) is None
sub11_near = sv.build_sub_map(
    m11, boxes11, targets11,
    [False], [False, False],
    0, 0,
)
assert sv.sokoban_bfs_single(sub11_near, (5, 2), boxes11[0], targets11[0]) is not None
print("Pass-through target candidate is rejected; current target remains valid.")
print("\n=============================")
print("所有测试通过！")
