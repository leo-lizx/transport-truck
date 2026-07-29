"""第二关共线目标在整车 PC 时间线中的单快照盲扫回归。"""

import sokoban_car_sim as car_sim
import sokoban_validator as sv


def _line_level(block_entries: bool = False) -> dict:
    game_map = sv._make_empty_map()
    for col in range(7, 11):
        game_map[5][col] = sv.TARGET
    for row in (2, 4, 6, 8):
        game_map[row][2] = sv.BOX
    if block_entries:
        game_map[5][6] = sv.WALL
        game_map[5][11] = sv.WALL
    return {
        'map': game_map,
        'start': (5, 1),
        'exit': (5, 2),
        'zone': car_sim.ZONE_LEFT,
        'box_classes': [1, 2, 3, 4],
        'target_classes': [4, 2, 1, 3],
        'level': 2,
    }


def _vertical_staged_level() -> dict:
    game_map = sv._make_empty_map()
    for row in range(5, 9):
        game_map[row][7] = sv.TARGET
    for row, col in ((3, 9), (4, 4), (4, 7), (7, 5)):
        game_map[row][col] = sv.BOX
    return {
        'map': game_map,
        'start': (5, 1),
        'exit': (5, 2),
        'zone': car_sim.ZONE_LEFT,
        'box_classes': [1, 2, 2, 2],
        'target_classes': [1, 2, 2, 2],
        'level': 2,
    }


def test_car_sim_executes_one_snapshot_line_sweep() -> None:
    frames, log = car_sim.build_timeline(_line_level())

    assert frames[-1]['stage'] == 'DONE'
    assert frames[-1]['win']
    assert frames[-1]['boxes'] == []
    assert sv.extract_elements(frames[-1]['base'], sv.TARGET) == []
    assert sum(frame.get('event') == 'FACE' for frame in frames) == 0
    assert sum(frame.get('event') == 'LINE_SWEEP_MATCH'
               for frame in frames) == 4
    first_line_frame = next(
        idx for idx, frame in enumerate(frames) if frame.get('line_sweep'))
    assert all(frame.get('line_sweep')
               for frame in frames[first_line_frame:])
    assert any('1 次读图 / 1 个执行批次' in line for line in log)


def test_car_sim_falls_back_before_execution() -> None:
    sim = car_sim.CarSim(_line_level(block_entries=True))
    sim.launch()
    initial_boxes = list(sim.boxes)
    sim.recognize()

    assert not sim.line_sweep_mode
    assert sim.line_sweep_plan is None
    assert sim.boxes == initial_boxes
    assert sim.mapping == [2, 1, 3, 0]
    assert sum(frame.get('event') == 'FACE' for frame in sim.frames) == 8
    assert any('执行前回退常规数字识别' in line for line in sim.log)


def test_directional_staging_does_not_add_virtual_walls() -> None:
    game_map = sv._make_empty_map()
    for col in range(7, 11):
        game_map[5][col] = sv.TARGET
    game_map[4][2] = sv.BOX
    game_map[5][6] = sv.WALL  # 强制使用右侧入口，复现旧守卫墙绕行情形
    original = [row[:] for row in game_map]
    line_info = sv.detect_stage2_line_targets(game_map)

    problem = sv.build_stage2_line_sweep_problem(
        game_map, (5, 2), line_info, 0, positive_direction=False)
    assert problem is not None
    assert game_map == original
    assert problem['map'] == original
    assert problem['map'] is game_map

    actions = sv.sokoban_bfs_single(
        problem['map'], (5, 2), problem['box_pos'],
        problem['staging'], problem['sweep_action'])
    assert actions is not None
    push_flags = sv._push_flags(actions, (5, 2), problem['box_pos'])
    push_actions = [action for action, pushed in zip(actions, push_flags)
                    if pushed]
    trace = sv._planned_box_trace(
        actions, push_flags, (5, 2), problem['box_pos'])

    assert push_actions[-1] == sv.DIR_LETTER.index('L')
    assert all(row in (4, 5) for row, _ in trace)


def test_staged_box_and_duplicate_classes_use_line_sweep() -> None:
    info = _vertical_staged_level()
    line_info = sv.detect_stage2_line_targets(info['map'])
    plan = sv.solve_stage2_line_sweep(
        info['map'], info['exit'], line_info,
        info['box_classes'], info['target_classes'])

    assert plan is not None
    assert plan['sub_solutions'][0]['box_start'] == (4, 7)
    assert plan['sub_solutions'][0]['direct_staging']
    assert not any(plan['sub_solutions'][0]['push_flags'][:-4])

    frames, _ = car_sim.build_timeline(info)
    assert frames[-1]['stage'] == 'DONE'
    assert frames[-1]['win']
    assert sum(frame.get('event') == 'LINE_SWEEP_MATCH'
               for frame in frames) == 4


def test_aligned_box_pushes_straight_to_staging() -> None:
    game_map = sv._make_empty_map()
    for col in range(7, 11):
        game_map[5][col] = sv.TARGET
    game_map[5][3] = sv.BOX
    original = [row[:] for row in game_map]
    problem = sv.build_stage2_line_sweep_problem(
        game_map, (4, 2), sv.detect_stage2_line_targets(game_map),
        0, positive_direction=True)

    assert problem is not None
    actions = sv.sokoban_bfs_single(
        problem['map'], (4, 2), problem['box_pos'],
        problem['staging'], problem['sweep_action'])
    assert actions is not None
    flags = sv._push_flags(actions, (4, 2), problem['box_pos'])
    pushes = [action for action, pushed in zip(actions, flags) if pushed]

    assert pushes == [sv.DIR_LETTER.index('R')] * 3
    assert game_map == original
