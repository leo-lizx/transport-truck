"""第二/三关部分匹配优先执行与地图重读回归。"""

import sokoban_validator as sv


def test_zero_match_requests_rescan() -> None:
    game_map = sv._make_empty_map()
    game_map[3][3] = sv.BOX
    game_map[7][3] = sv.BOX
    game_map[3][10] = sv.TARGET
    game_map[7][10] = sv.TARGET

    scout = sv.plan_scout_phase_v2(
        game_map, (5, 2),
        box_classes=[1, 2],
        target_classes=[9, 9],
    )
    assert scout['all_visited']
    assert scout['matched_count'] == 0
    assert scout['box_to_target_idx'] == [
        sv.UNMATCHED_TARGET, sv.UNMATCHED_TARGET]

    batch = sv.solve_recognized_batch(
        2, game_map, scout['player_after_scout'],
        scout['box_to_target_idx'])
    assert batch is not None
    assert batch['status'] == 'rescan_without_push'
    assert batch['needs_rescan']


def test_stage2_pushes_partial_match_then_rescans() -> None:
    game_map = sv._make_empty_map()
    game_map[3][3] = sv.BOX
    game_map[7][3] = sv.BOX
    game_map[3][10] = sv.TARGET
    game_map[7][10] = sv.TARGET

    scout = sv.plan_scout_phase_v2(
        game_map, (5, 2),
        box_classes=[1, 2],
        target_classes=[1, 9],
    )
    assert scout['all_visited']
    assert scout['matched_count'] == 1
    assert scout['box_to_target_idx'] == [0, sv.UNMATCHED_TARGET]

    problem = sv.build_partial_stage2_problem(
        game_map, scout['box_to_target_idx'])
    assert problem is not None
    assert problem['map'][7][3] == sv.WALL
    assert problem['map'][7][10] == sv.EMPTY

    batch = sv.solve_recognized_batch(
        2, game_map, scout['player_after_scout'],
        scout['box_to_target_idx'])
    assert batch is not None
    assert batch['status'] == 'pushed'
    assert batch['matched_count'] == 1
    assert batch['needs_rescan']
    assert len(batch['push_result']['sub_solutions']) == 1


def test_stage3_partial_match_keeps_bomb_replanning() -> None:
    game_map = sv._make_empty_map()
    for row in range(1, 11):
        game_map[row][7] = sv.WALL
    game_map[4][3] = sv.BOX
    game_map[8][3] = sv.BOX
    game_map[4][10] = sv.TARGET
    game_map[8][10] = sv.TARGET
    game_map[5][5] = sv.BOMB

    batch = sv.solve_recognized_batch(
        3, game_map, (6, 2),
        [0, sv.UNMATCHED_TARGET])
    assert batch is not None
    assert batch['status'] == 'pushed'
    assert batch['matched_count'] == 1
    assert batch['needs_rescan']
    assert any(phase['kind'] == 'bomb' for phase in batch['phases'])
    assert batch['phases'][-1]['kind'] == 'push'


def test_remaining_single_box_skips_recognition_in_stage2() -> None:
    game_map = sv._make_empty_map()
    game_map[5][5] = sv.BOX
    game_map[5][8] = sv.TARGET

    mapping = sv.mapping_for_remaining_rescan(game_map)
    assert mapping == [0]

    batch = sv.solve_recognized_batch(2, game_map, (5, 4), mapping)
    assert batch is not None
    assert batch['status'] == 'pushed'
    assert batch['matched_count'] == 1
    assert not batch['needs_rescan']


def test_remaining_single_box_keeps_stage3_bomb_replanning() -> None:
    game_map = sv._make_empty_map()
    for row in range(1, 11):
        game_map[row][7] = sv.WALL
    game_map[4][3] = sv.BOX
    game_map[4][10] = sv.TARGET
    game_map[5][5] = sv.BOMB

    mapping = sv.mapping_for_remaining_rescan(game_map)
    assert mapping == [0]

    batch = sv.solve_recognized_batch(3, game_map, (6, 2), mapping)
    assert batch is not None
    assert batch['status'] == 'pushed'
    assert not batch['needs_rescan']
    assert any(phase['kind'] == 'bomb' for phase in batch['phases'])
    assert batch['phases'][-1]['kind'] == 'push'


def test_remaining_rescan_requires_exactly_one_box_and_target() -> None:
    game_map = sv._make_empty_map()
    game_map[5][5] = sv.BOX
    assert sv.mapping_for_remaining_rescan(game_map) is None

    game_map[5][8] = sv.TARGET
    game_map[7][5] = sv.BOX
    game_map[7][8] = sv.TARGET
    assert sv.mapping_for_remaining_rescan(game_map) is None


if __name__ == '__main__':
    test_zero_match_requests_rescan()
    test_stage2_pushes_partial_match_then_rescans()
    test_stage3_partial_match_keeps_bomb_replanning()
    test_remaining_single_box_skips_recognition_in_stage2()
    test_remaining_single_box_keeps_stage3_bomb_replanning()
    test_remaining_rescan_requires_exactly_one_box_and_target()
    print('partial recognition retry tests passed')
