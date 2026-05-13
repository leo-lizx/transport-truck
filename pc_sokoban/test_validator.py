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
_, final_box = sv.simulate_actions(sol, (5, 2), (3, 3))
assert final_box == (3, 10), f"箱子最终位置错误: {final_box}"
print("通关验证: OK")

# 测试2: Stage1 贪心
print("\n=== 测试2: Stage1 贪心（2箱） ===")
m2 = sv._make_empty_map()
m2[2][2] = sv.BOX
m2[2][12] = sv.BOX
m2[8][2] = sv.TARGET
m2[8][12] = sv.TARGET
r = sv.solve_stage1(m2, (5, 7))
assert r is not None, "Stage1 应有解"
print(f"总步数: {r['total_steps']}  子任务数: {len(r['sub_solutions'])}")

# 测试3: 地图自动生成
print("\n=== 测试3: 地图生成 ===")
for seed in [1, 2, 42, 100]:
    m3, p3 = sv.generate_map(stage=1, box_count=2, seed=seed)
    boxes = sv.extract_elements(m3, sv.BOX)
    tgts = sv.extract_elements(m3, sv.TARGET)
    r3 = sv.solve_stage1(m3, p3)
    status = f"OK steps={r3['total_steps']}" if r3 else "UNSOLVABLE"
    print(f"  seed={seed}: boxes={len(boxes)} tgts={len(tgts)} -> {status}")

# 测试4: 路点压缩
print("\n=== 测试4: 路点压缩 ===")
acts = [3, 3, 3, 1, 1, 2, 2]  # RRR DD LL
wpts = sv.actions_to_waypoints(acts, (5, 2))
print(f"动作: {acts}  路点数: {len(wpts)}")
assert len(wpts) == 3, f"期望3个路点，实际{len(wpts)}"

# 测试5: 死局检测
print("\n=== 测试5: 死局检测 ===")
m5 = sv._make_empty_map()
m5[1][1] = sv.BOX  # 左上角 = 死局
dead, dp = sv.check_deadlock(m5)
assert dead, "角落箱子应检测为死局"
print(f"死局箱子: ({dp[1]},{dp[0]}) OK")

print("\n=============================")
print("所有测试通过！")
