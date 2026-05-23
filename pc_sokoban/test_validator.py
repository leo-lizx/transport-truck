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

# 测试8: 排除法 (V2 侦查) — N-1 推 1
print("\n=== 测试8: 排除法 (V2 侦查) ===")
m8, p8 = sv.generate_map(stage=2, box_count=3, seed=11)
# 先跑无推断
no_inf = sv.plan_scout_phase_v2(m8, p8, use_inference=False)
inf    = sv.plan_scout_phase_v2(m8, p8, use_inference=True)
total_items = 6  # 3 box + 3 target
# 排除法: 至少最后一个 box 和最后一个 target 都能被推断 → real ≤ 4
print(f"  无推断: real={no_inf['visited_real']} inferred={no_inf['visited_inferred']} (期望 6/0)")
print(f"  开推断: real={inf['visited_real']}    inferred={inf['visited_inferred']}")
assert no_inf['visited_real'] == total_items, "无推断时应实测全部"
assert inf['visited_real'] < total_items, "推断时应少于全量"
assert inf['all_visited'], "推断后应全部 resolved"
assert inf['box_to_target_idx'] is not None
print(f"  推断结果 box_classes={inf['box_classes']} target_classes={inf['target_classes']}")
print(f"  box→target 映射: {inf['box_to_target_idx']}")

# 测试9: V2 + Stage2 完整求解 (走真实路径)
print("\n=== 测试9: V2 完整求解 ===")
sol9 = sv.solve_level(2, m8, p8, use_inference=True)
assert sol9 is not None and sol9['scout']['all_visited']
print(f"  总步数(开推断): {sol9['total_steps']}  侦查实测+推断: "
      f"{sol9['scout']['visited_real']}+{sol9['scout']['visited_inferred']}")
sol9b = sv.solve_level(2, m8, p8, use_inference=False)
print(f"  总步数(无推断): {sol9b['total_steps']}")
assert sol9['total_steps'] <= sol9b['total_steps'], "推断应不慢于无推断"

# Test 10: half-grid launch start should not be truncated to the blocked lower row.
print("\n=== Test 10: half-grid launch start ===")
m10 = sv._make_empty_map()
m10[5][1] = sv.WALL
m10[4][4] = sv.BOX
m10[4][8] = sv.TARGET
r10 = sv.solve_stage1_from_float_start(m10, (5.5, 1.0), preferred_start=(6, 1))
assert r10 is not None, "half-grid start should pick a reachable integer entry cell"
assert r10['entry_cell'] == (6, 1), f"expected entry (6,1), got {r10['entry_cell']}"
assert abs(r10['entry_distance_m'] - (0.5 * sv.GRID_STEP_Y_M)) < 1e-6
assert r10['first_sub_waypoints'][0] == (6, 1), "first waypoint sent to chassis must be a definite grid cell"

print("\n=============================")
print("所有测试通过！")
