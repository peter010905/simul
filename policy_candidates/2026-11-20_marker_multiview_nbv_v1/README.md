# Mixed · Marker-Aware Multi-View NBV v1

실험 버전 이름은 `marker_multiview_nbv_v1`이다. 기존 `2026-11-20_mixed_adaptive_faces`가 대조군이며 가중치와 기본 실행 경로를 그대로 보존한다. 이 디렉터리는 해당 Mixed의 Python 패키지에만 적용하는 별도 오버레이이다.

[Gazebo 결과와 정량 비교](RESULTS.md)

## 기존 Mixed가 결정하는 순서

|단계|기존 코드와 판정|
|---|---|
|확정 ID와 목표 우선순위|기존 `phased_hybrid.py`의 `PhasedHybrid.select`는 이미 선택된 `approach_id`, `focus_id`를 유지한다. 그다음 3초 이내이고 map frame이 일치하는 global costmap을 확인하고 익명 RGB 목표, 확인한 상자 및 일반 표면, 기존 Flow 선택 순으로 진행한다.|
|Frontier|기존 `center_wall.py`의 `discovery_frontier`는 정보 이득 0.15 미만, 제자리 후보, Nav2 연결영역 밖, 최근 실패 목표를 제외한다. 남은 후보는 정보 이득을 이동거리·짧은 재이동·재방문 페널티로 나눈다. 벽 단계도 새 내부 물체가 생기면 중단할 수 있다.|
|면 후보|기존 `adaptive_faces.py`의 `inspection_pose`는 면 법선으로 0.40–1.50 m, 접선으로 ±0.50 m를 탐색한다. 면의 양 끝, SLAM 시선, 0.32 m 여유, 유한 보행거리, Nav2 costmap 연결성을 검사한다. `CenterWall.decide`는 새로 확인된 각 면에서 한 위치씩 구한 뒤 보행거리가 최소인 면을 선택한다. 두 면이 동시에 보일 이득이 이 비교에는 없다.|
|RGB·면 기억|설치된 `hybrid.py:22–118`의 `publish_print`와 `PrintMemory`는 RGB 윤곽을 노출 시점 TF와 LiDAR로 odom에 투영한다. 측정 거리가 없는 후보는 위치 추적에서 빠진다. 세 번 연속 검출한 후보에 제자리 또는 접근 관찰을 제안한다. 기존 `two_views.py`의 `TwoViews.observe`가 카메라·레이저와 서로 다른 관측 각도로 확인한 상자 면을 기억하고, `edge_policy.py`도 입사각·판독 가능 거리를 고려해 실제 카메라 커버리지를 기록한다. 기존 상자 선택은 관찰 완료 각도는 이용하지만 그 면의 최고 관찰 품질을 연속값으로 비교하지 않는다.|
|Nav2·복구|설치된 `mission.py:409,487,628,677,841`이 취소·결과·실행을 소유한다. 인식 힌트로 취소한 경우 정착 후 재계획하고, 정체·시간 초과·범퍼·스핀 실패는 기존 복구 분기를 따른다. Nav2의 `NavigateToPose`, `Spin`, 후방 여유를 확인하는 `BackUp`을 사용한다. 후보의 격자 연결성은 실제 Nav2 경로 성공 보증이 아니다.|

## v1 결정과 관찰

- `nbv_faces.py`는 확인된 상자 면과 SLAM에서 얻은 표면을 odom의 위치·법선으로 연결한다. 실제 노출에서 얻은 카메라 내부 파라미터와 base→camera 외부 파라미터를 다음 자세의 카메라 예측에도 재사용한다. 앞면 입사각, FOV 안에서 중앙·양쪽 세 지점으로 향하는 자유 시선, 미지 셀, 8 cm 보수적 마커 크기 가정에 따른 예상 픽셀 수로 각 면의 품질을 계산한다. 각 자세·방향에서 개별 면의 `max(0, 예상 품질 − 실측 최고 품질)`을 합하고 격자 이동거리와 회전에 비용을 부과한다. CPU 예산은 가까운 14개 기본 면, 중앙·좌·우 위치 및 세 개의 방향으로 제한한다. 관찰할 면 전체에 대한 전역 최적화는 수행하지 않는다.
- 관찰 품질을 올리려면 실제 처리된 RGB 노출 시각의 TF와 시간 동기된 LiDAR가 필요하다. 이 점수는 면이 보일 기회를 나타낸다. 마커 등록은 기존 ArUco 디코더가 한다. 같은 면의 기본 목표는 최대 두 번, 간격은 최소 45초다.
- `nbv_camera.py`와 `nbv_rgb.py`가 윤곽의 정규화 영상 위치, 픽셀 크기, 광학 레이의 odom 방위, 선택적 LiDAR 위치를 전달한다. 같은 영상에서 판독된 사각형은 익명 추적에서 제외한다. 추적별 신뢰도, 누적 관찰 수, 품질, 마지막 시각과 거리 추정치를 보관한다. 각도만 있는 후보에는 가까운 관측 원점에서 최대 한 번 저비용 시선 재정렬을 허용한다. 세 번 이상 확인했고 거리·충돌 여유·가시성이 확보된 후보에 한해 평행 이동을 비교한다. 두 행동의 예상 관찰 이득을 이동·회전 비용으로 나눠 비교한다. 신뢰도는 휴리스틱 점수다.
- `marker_nbv.py`와 `stage.py`는 익명 후보의 등장·소실·신뢰 단계 상승 및 ID 등록을 재평가 신호로 쓴다. 동일 목표 중복 전송을 억제하고 일반 RGB 전환에는 목표 유지 4초, 점수 차 0.18을 적용한다. 확인된 ID, 위험 및 복구 분기는 기존 소유자에게 둔다. 안전 상태라면 익명 면을 향해 주행 중에도 다른 후보를 비교할 수 있다. 실제 노출 없이 의사결정 로그가 면 품질이나 ID를 올리지 않는다.

## 재현

시뮬 VM에서 `stage.py`에 **기존 설치 소스의 osp_mission Python 디렉터리**와 **존재하지 않는 출력 디렉터리**를 준다. 핵심 원본 SHA-256을 검사한 뒤 새 디렉터리에서만 변경한다. 기존 adaptive workspace를 source한 뒤 새 오버레이를 PYTHONPATH 맨 앞에 둔다. `python3 -m unittest -v test_nbv test_guarded test_camera`로 회귀 검사를 실행한다. `test_camera`는 Original 사전에서 ID가 나오지 않는 합성 사각형을 RGB 픽셀로 주고, 거리 없는 첫 프레임의 제자리 관찰 제안과 반복 거리 측정 뒤의 접근 제안까지 통과하는지 확인한다. `replay.sh` 인수는 기존 recording, 시뮬 전용 ROS domain, 새 출력 경로, 오버레이 순이다. 기존 실험 하네스와 같은 `.domain.lock`을 잡은 상태에서 실행한다. `evaluate.py BASE VERSION`은 실제 월드와 출발 설정 해시가 같은지 확인한다. 지도·시뮬 진실값·정답 ID 목록은 평가기만 읽는다.

설치 원본의 VM 경로는 `~/flow_v3_r32_adaptive_ws/src/osp_mission/osp_mission/`이다. 예를 들어 이미 설치된 VM에서 아래 명령은 별도 출력 폴더에만 기록한다. 셸은 Bash를 사용한다.

```bash
cd ~/marker-nbv-v1/frozen
flock "$HOME/track-recordings/.domain.lock" bash replay.sh \
  "$HOME/track-recordings/adaptive-track-4KJqzbo1" 174 \
  "$PWD/results/marker-nbv-track-repeat-$(date +%s)" "$PWD/overlay"
```

### 지표 정의

- 완주는 7·32·103 세 ID를 모두 등록하고 Gazebo 진실 궤적에서 각각 1.5 m 이내에 한 번 이상 진입한 시각이다. 시간 제한 종료는 미완주로 표시한다. 거리는 Gazebo 평면 궤적을 5 Hz로 보간해 누적한 기준이다. 범퍼는 접촉 시작 횟수다.
- 목표 전환은 미션이 실제 보낸 연속 Nav2 목적지 사이의 평면 이동이 0.15 m를 넘은 횟수다. 완료 후 새 목적지도 포함한다. `perception_preemptions_to_different_goal`은 지각 힌트로 기존 주행을 취소하고 실제 다른 목적지를 보낸 경우만 센다. 제자리 회전은 별도다.
- 익명 후보의 **접근**은 평행 이동을 선택한 경우다. 비생산적 접근의 대리지표는 출발 후 다음 정책 결정 3초 뒤까지 새 ID 등록이 없었던 횟수다. 진짜 마커에 접근해 판독에 실패할 수도 있으므로 영상 오탐과 구분한다. 후보 지점 좌표가 기록된 경우 동시각 로봇 map 자세와 Gazebo 자세로 world에 옮겨 모든 실제 마커와 0.50 m 넘게 떨어진 접근도 따로 센다. 위치 오차와 영상 오탐은 서로 다른 지표다.
- 거리·각도·관찰 품질의 모델은 수평 2D다. 중앙·양옆 세 광선은 부분 가림의 근사이며 색 번짐, 모션 블러, 마커가 붙은 실제 높이까지 예측하지 않는다. 1.5 m 통과와 ID 판독은 독립 관측이므로 같은 영상 프레임에서 1.5 m 판독을 보증하지 않는다.

## 기준 회귀 이후의 제한형 v1.1

`marker_nbv_local_v1_1`은 `guarded_nbv.py`를 쓰는 또 하나의 opt-in 버전이다. `stage.py --guarded`로 별도 디렉터리를 만든다. 최종 설치 위치는 VM의 `~/marker-nbv-v1/release-v1_1/overlay`다.

- 일반 이동, Frontier, 확인 ID, 벽→내부 선점 판정은 기존 스케줄러가 선택한 목적지를 기준으로 삼는다. 면을 보기 위한 기본 목적지가 있을 때 그 주변에서만 카메라 자세를 비교한다. 후보의 최대 평면 변위는 0.324 m, 늘어날 수 있는 격자 경로 길이는 0.25 m다. 기존 목적지에서 바라본 주면 품질을 0.035 넘게 희생하는 대안은 거절한다.
- 같은 평면에서 법선 일치도·평면 간 거리·접선 거리 조건이 맞는 가까운 패치는 최고 이득 하나로 계산한다. 서로 떨어진 평행면을 같은 면으로 묶을 수 있는 보수적인 근사다. 기본 자세의 정보이득보다 비용 차감 후 0.15를 초과해 나아진 두 면 이상 후보만 채택한다.
- 이미 낮은 품질로 관찰한 면은 현재 위치에서 0.12–0.45 rad만 돌면 품질이 0.25 넘게 좋아질 때 제자리에서 다시 볼 수 있다. 추가 회전 사이에는 45초를 두고 마지막 60초는 기존 주행에 남긴다. 범퍼와 회전 안전 확인은 기존 Nav2 동작을 거친다.
- 거리 없는 익명 후보의 제자리 확인, 측정된 RGB 후보의 접근, ID 우선순위, 취소 히스테리시스는 v1과 같다. 새 모델은 학습을 수행하지 않고 기존 가중치 위의 관찰·목표 선택 방식을 바꾼다.

```bash
cd ~/marker-nbv-v1/release-v1_1
NBV_VERSION=marker_nbv_local_v1_1 flock "$HOME/track-recordings/.domain.lock" bash replay.sh \
  "$HOME/track-recordings/adaptive-track-4KJqzbo1" 174 \
  "$PWD/results/nbv-local-repeat-$(date +%s)" "$PWD/overlay"
```

면 좌표가 없는 Flow frontier에 관찰 회전만 붙은 경우도 있다. 기준 주행 및 첫 랜덤 비교에 동결했던 v1.1에서는 이 입력이 예외 처리 경로를 거쳐 원래 Flow 목표로 복귀했다. 최종 패키지는 이 경우 명시적으로 같은 Flow 선택을 반환하도록 위임한다. 실제 Nav2에 전달되는 목표는 양쪽 경로에서 동일하다. 이 입력에 대한 회귀 테스트를 포함한다.

### 근거 코드

- 기존 관찰점과 단계별 목표 우선순위는 [adaptive_faces.py](../2026-11-20_mixed_adaptive_faces/adaptive_faces.py), [center_wall.py](../2026-11-20_mixed_adaptive_faces/center_wall.py), [phased_hybrid.py](../2026-11-20_mixed_adaptive_faces/phased_hybrid.py)에 있다. 실행 중인 설치본 기준 네이티브 포인트는 `mission.py::tick`, `mission.py::observe_camera`, `hybrid.py::PrintMemory.choose`, `detector.py::on_rgb`다. 검사한 설치본의 SHA-256과 변경 진입점은 [stage.py](stage.py)에 있다.
- 새 동작은 [면 품질·가림](nbv_faces.py), [제한형 관찰 자세 비교](guarded_nbv.py), [RGB 영상 투영](nbv_camera.py), [후보 추적·비용 비교](nbv_rgb.py), [목표 재평가](marker_nbv.py)에서 추적할 수 있다.
