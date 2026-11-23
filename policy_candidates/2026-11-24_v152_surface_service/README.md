# Mixed v1.5.2.22 · Surface Service

640×480·5fps 시험장 시뮬레이션 후보. 기준 버전은 실물 대시보드에서 Focus Continuity로 표시되는 `v152_focus_continuity_box1122_ws`다. 설치기는 원본에서 별도 작업공간을 생성한다.

## 목표 선택

- Nav2가 이동을 마치거나 기존 안전·마커 이벤트가 발생하면 다음 행동을 결정한다. 동일 ID의 검증된 접근과 센서 이상·접촉 처리는 부모 정책의 순서를 따른다.
- 일반 벽 조각 수가 많으면 같은 방면의 짧은 관찰점이 수십 번 연속해서 선택될 수 있었다. 새 상자 면이나 유효한 미탐색 경계가 있을 때는 기존 벽 관찰 예약을 통해 전용 벽 목표를 선택한다. 그 밖의 벽은 이동 중 카메라로 볼 수 있는 정보로 계속 평가한다. 갈 수 있는 일이 벽뿐이면 일반 벽 목표를 모두 열어 둔다.
- 같은 상자에서 새로 볼 수 있는 안전한 면을 기존 지역 방문 시간 안에서 이어서 본다. 반경 2.2m, 최대 4회 또는 55초로 한 번의 방문을 제한한다. 카메라가 보지 못한 면은 제한에 도달해도 미확인 상태다. 위험·검증된 마커·장시간 미처리된 벽 작업은 먼저 처리할 수 있다.
- 벽 목표 후보를 압축할 때 서로 다른 표면 법선을 대표하는 구간을 먼저 남긴다. 어느 쪽을 최근 보았는지는 촬영 조건을 통과해 관찰 이력에 새로 들어온 벽 구간으로 갱신한다. 상자를 보러 가며 확보한 관찰도 여기에 포함된다. 목표를 보내는 시각은 재시도 제한에만 사용한다.
- 벽 관찰점마다 안전 격자와 Nav2 비용지도를 검사한다. 허용되는 위치들을 원래 비용 순으로 검사해 가림·차단 이력이 없는 첫 위치를 사용한다. 안전한 SLAM 격자상의 보행 길이는 예상 이동 비용이며, 실제 경로는 Nav2가 결정한다.

## 실행

```sh
python3 policy_candidates/2026-11-24_v152_surface_service/install.py \
  ~/v152_focus_continuity_box1122_ws ~/focus_surface_service_test_ws
cd ~/focus_surface_service_test_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash
```

[시험장 실행기](../2026-11-24_v152_regional_survey/run_exam_640.py)에 사용 중인 Jazzy 시험장의 `run_archived.py`, 새 작업공간, 고정한 시험장 파일과 시드를 전달한다. 시뮬레이션은 RGB, LiDAR, SLAM과 실시간 비용지도를 사용한다. 마커의 정답 위치는 독립 채점에서 사용한다. [범위가 없는 ID 재관찰](../2026-11-24_v152_bearing_advance/README.md)과 [읽을 수 있는 벽 거리 계산](../2026-11-24_v152_wall_evidence/README.md)은 부모 후보에서 적용했다.
