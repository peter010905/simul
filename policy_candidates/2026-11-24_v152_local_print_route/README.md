# Mixed v1.5.2.23 · Local Print Route

Focus Continuity의 구역 탐색과 벽 관찰에 [Surface Service](../2026-11-24_v152_surface_service/README.md)를 사용한다. 추가 수정은 익명 인쇄물 후보로 잠깐 이탈하는 경로에 한정한다.

`Grid.reachable`의 거리는 미터다. 이전 `PrintMemory.choose`는 여기에 격자 해상도를 다시 곱해서 5cm 격자에서 이동 비용을 20분의 1로 계산했다. SLAM 지도와 Nav2에서 각각 멀리 돌아 연결되는 위치도 짧은 이탈로 인정될 수 있었다.

세 번 반복 관측한 익명 후보에는 기존 2.4m 예산을 실제 격자 보행 거리로 적용한다. 안전한 SLAM 셀과 Nav2에서 출발점에 연결된 셀을 함께 통과하는 짧은 경로가 있어야 한다. 정면만 확인하면 되는 경우 현재 위치에서 보는 동작도 허용한다. 이 경로 길이는 실제 Nav2 주행 전의 추정치이며, 주행 로그의 `hybrid_goals.jsonl`에 `print_route_m`로 남긴다. 탐사 중 확정한 ID의 최종 접근과 카메라 관찰 증거는 원래 판정기를 사용한다.

서로 다른 3가지 지도 해상도에서 먼 후보, 짧은 정상 경로, 우회가 긴 비용지도 장벽, 제자리 관찰, 시작 셀 차단을 검사한다.

```sh
python3 policy_candidates/2026-11-24_v152_local_print_route/install.py \
  ~/v152_focus_continuity_box1122_ws ~/focus_local_print_route_ws
cd ~/focus_local_print_route_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash
```
