# Mixed · 이동 중 옆면 관찰

기존 `flow_v3_r32_corner_ws`를 보존하고 `flow_v3_r32_side_ws`를 별도로 설치했다. 벽이나 상자 면을 향해 주행 중일 때, 현재 경로 옆의 아직 보지 않은 면이 카메라 1.2m 이내에 들어오면 Nav2 목표를 안전하게 취소하고 짧게 회전·관찰한 뒤 원래 이동을 재개한다. 마커 ID가 확정된 접근, ID 후보 집중 관찰, 인쇄 테두리 재접근은 중간 관찰에서 제외한다. 기존 `NearPassInspection`의 최대 8회·총 35초·15초 간격·남은 시간 45초 조건을 유지한다. Nav2 회전과 충돌 검사를 그대로 사용한다.

## 코드에서 확인한 정책 충돌

| 항목 | 기존 동작 | 영향 | 상태 |
| --- | --- | --- | --- |
| `scripts/robot_session.py`의 launch 옵션 | `allow_mid_route_wall_inspection:=false` | `NearPassRuntime.request()`가 주행 중 호출되지 않음 | Confirmed |
| `osp_mission/mission.py`의 `protected_navigation` | `box_point`가 있는 면 목표까지 중간 관찰 금지 | 위 옵션만 켜도 상자 면으로 가는 동안 옆면을 지나침 | Confirmed |
| `Hybrid.should_interrupt()` | LiDAR 위치가 일치한 인쇄 후보 3회가 최근 1초 안에 있어야 개입 | 짧게 스친 측면 마커는 다음 목표까지 기다림 | Confirmed |
| `PassingGlance` | 가장자리 인쇄형 후보만 허용하고 회전 요청을 ±6°로 제한 | 옆으로 크게 놓인 면을 정면에 넣기는 어려움 | Confirmed |
| 목표당 고정 관찰 방향 | Flow/상자 목표는 도착면을 향한 yaw로 보냄 | 이동 경로 옆면을 보는 회전은 별도 정책이 필요 | Confirmed |
| 카메라 해상도·LiDAR 가림 | 먼 측면 마커가 영상에서 충분히 보였는지 별도 측정 필요 | 실제 놓친 개수는 미확정 | Hypothesis |

최근 실물 실행 `20261119T110237-11c9096872`은 약 607초 동안 목표 40개를 보냈다(`flow` 27, `box_face_view` 7, `detour` 4, `print` 1, `approach` 1). 옆면 관찰 로그는 **0건**이고, 원본 bag에는 익명 인쇄 후보 27건·익명 가장자리 회전 요청 1건이 있다. 저장된 결정 지도 40장에 실제 카메라 보정값을 근사 적용한 오프라인 검사에서 옆면 관찰 후보가 2장에 나타났다. 지도 결정 시점만 검사했으므로 전체 주행 중 기회 수나 발견 성공률은 아니다.

마지막 `stop_unverified`는 종료 신호 뒤 정지 검사에서 기록됐다. 뒤이은 정지 명령과 읽기 전용 오도메트리는 정지를 확인했으나 기존 상태 파일은 갱신되지 않았다. 새 공통 실행기는 0속도를 보내면서 최근 1초의 안정된 오도메트리가 확인되면 즉시 정지 완료로 처리하고, 뒤의 정지 명령이 성공하면 이전 상태 파일도 `stopped`로 갱신한다. 속도 허용 기준은 바꾸지 않았다.

## 검증과 한계

- 새 ROS 작업공간 빌드와 `Mission`·`Hybrid` 로드 성공. 측면 목표 분류 2개 검사 통과.
- 보드 제어 23개, 실행기 26개 검사 통과. VM에서 `dry-run-side`가 올바른 작업공간과 `allow_mid_route_wall_inspection:=true`를 선택함을 확인했다.
- 기존 주행은 보존했다. 새 정책의 실물 주행, 마커 추가 발견, 시간 증가, 범퍼 횟수는 **미측정**이다. 로봇은 도크에 있고 정지 확인 완료 상태라 출발시키지 않았다.

## 재현

`stage.py`로 `flow_v3_r32_corner_ws/src/osp_mission/osp_mission`에서 후보를 생성하고, `stage_workspace.py`로 별도 `flow_v3_r32_side_ws`에 설치한다. `colcon build --packages-select osp_mission --symlink-install` 후 `test_side_route.py`를 실행한다. 실물 준비 검사는 `./robot.sh check-side`, 명시적 주행은 `./robot.sh start-side` 또는 보드의 **Mixed · 이동 중 옆면 관찰** 버튼이다.
