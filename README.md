# OSP Final Project — ROS 2 Jazzy

[**시뮬레이션 영상 다운로드 — GitHub Release**](https://github.com/peter010905/simul/releases/tag/simulation-videos-2026-11)

비공개 저장소 권한이 있는 GitHub 계정으로 로그인하면 영상을 받을 수 있습니다.
전체 영상 목록과 당시 평가 결과는 [영상 메타데이터](repository_snapshot/videos.json)에 기록합니다.


> **2026-11-06 업로드 기준:** 아래 과거 기록과 별도로 최신 수정본은
> `training_robust/native90/runtime_fix_v13_observation_staging/src/osp_mission`에 있습니다.
> 제자리 회전 복구, ID 인식과 독립적인 **1.5m 접근 판정**, 동일 위치의 상충 ID 처리,
> 미관측 면 재탐색을 반영했으며 테스트 **189개**를 통과했습니다.
> 실제 Gazebo 회귀 검증은 진행 중입니다. 기본 `src/osp_mission`과 최신 후보를 구분해 사용하세요.
> [업로드 시점 상태](repository_snapshot/status.json) ·
> [수정 내역](repository_snapshot/implementation.json) ·
> [실패 분석](training_robust/native_diagnostics/native_search_v1_failure_analysis/report_ko.txt)
>
> 저장소에는 소스·시나리오·정책·검증 요약을 포함합니다. 영상, SLAM pose graph,
> 빌드 결과와 전체 실행 로그는 로컬에 보관합니다. 기존 실험 JSON의 `/home/peter/simul` 경로는
> 실험 당시 원본 경로이며, 다른 컴퓨터에서는 작업 경로에 맞춰 설정해야 합니다.

최신 후보를 별도 빌드하려면 Ubuntu/ROS 2 Jazzy 의존성을 설치한 뒤:

```bash
cd training_robust/native90/runtime_fix_v13_observation_staging
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/osp_mission/test -q
```


TurtleBot4 Standard / Gazebo Harmonic / Nav2 / SLAM Toolbox.
목표: 시작 신호 이후 600초 동안 자율 탐색, Original ArUco 3개 인식 및 위치 표시, 지도와 pose graph 저장.

현재 기본 시연은 추가 장애물 12개와 서로 다른 장애물 뒷면의 마커 3개다. 학습 풀은 서로 다른 배치 400개이며 장애물 수 8/12/16/24/32개, 벽·장애물·뒷면 부착, 출발 자세, 지도 방향과 센서 누락을 섞는다. 32개는 요청 수이며 배치 제약으로 실제 31개인 장면도 있다. 모든 임무는 세 번째 마커를 찾거나 600초가 되면 종료한다.

최신 2D 개발 검증은 LiDAR 640개 빔·5cm 지도·5fps 카메라로 160개 배치를 비교했다. 기본 정책은 **120/160**, 면별 실험 후보는 **125/160**, 대칭 공간 후보는 **127/160**을 성공했고 접촉은 모두 0회였다. 대칭 후보는 장애물 32개·뒷면 조건과 평균 시간이 기본 정책보다 나빠 채택하지 않았다. 학습 배치 40개에서 나온 87.5%도 다른 40개에서는 72.5%로 내려갔다. 이 수치는 정책 개발용이며 독립 시험이나 Gazebo 완주 결과가 아니다. 자세한 학습·검증 기록은 [TRAINING.md](TRAINING.md)에 있다.

2026-11-05 현재 **통신 권한은 해결**됐으며 실제 Gazebo에서 장애물 12개·뒷면 마커 3개를 **238.9초, 충돌 0회**, 좌표 오차 2.7–5.0cm로 완료했다. 이후 기존 정책의 장애물 8/16/24/32개 개발 시험은 **1/4 성공**이었다. 최신 속도·좌표 수정 후보는 실패했던 장애물 16개 배치를 **356.447초에 세 ID 발견, 충돌 0회, 좌표 오차 1.2–5.4cm**로 통과했고 세 목표 모두 2m 안쪽까지 접근했다. 실제 영상 1,783프레임과 지도·CSV 저장도 감사했다. 이는 한 개발 배치의 성공이며 다른 배치의 성능을 보장하지 않는다. 관측 기록을 유지하는 별도 후보는 16/24/32개 배치를 실행 중이다. 이전의 좌표 오차 실패 판정은 보존한다. **Gazebo 성공률 90%는 아직 미달이며, 예약한 독립 시험 40개는 미사용**이다. 기본 코드의 시작 준비·좌표 저장·얇은 벽 법선 수정은 테스트 149개와 빌드를 통과했다. 실험 후보는 별도 설치 경로에서 검증 중이다.

현재 PC의 Mesa NVK 그래픽 경로에서 GPU LiDAR의 넓은 방향이 무한대 거리로 누락되는 문제를 재현했다. 로봇 없는 사각형 방에서도 640개 중 347개만 측정됐고, 소프트웨어 렌더링에서는 640개 모두 측정되며 거리 오차 95백분위가 7.4mm였다. 시뮬레이션 기본값을 `render_backend:=software`로 바꾸고 실제 임무를 재검증 중이다. 시스템 드라이버는 변경하지 않는다. 진단 기록은 `training_robust/native_diagnostics/lidar_rendering/`에 있다.

## 설치와 빌드

Ubuntu 24.04와 ROS 2 Jazzy에서:

```bash
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-navigation ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-cv-bridge ros-jazzy-tf2-geometry-msgs python3-opencv python3-scipy python3-pytest
cd /home/peter/simul
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 시뮬레이션 실행

모든 터미널에서 같은 ROS_DOMAIN_ID를 사용한다. 아래 48은 로컬 시뮬레이션용이며 실제 로봇에서는 로봇의 값을 사용한다.

```bash
cd /home/peter/simul
source install/setup.bash
export ROS_DOMAIN_ID=48
bash run_gazebo_guarded.sh ros2 launch osp_mission simulation.launch.py
```

준비가 끝난 뒤 다른 터미널에서:

```bash
source /home/peter/simul/install/setup.bash
export ROS_DOMAIN_ID=48
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status
```

시작 요청은 map·TF·신선한 scan·카메라와 Nav2의 활성 상태를 확인한다. Gazebo에서는 바퀴 구동 제어기 활성화와 신선한 원시 바퀴 odom도 필요하다. 준비되지 않았으면 임무 시간을 시작하지 않는다. 시뮬레이션 기본값은 PDF에 맞춰 지정 위치·방향에서 바로 출발한다. 충전 도크 출발을 별도로 시험하려면 `start_docked:=true`를 지정하며, 이 경우 시작 신호 후 자동 undock한다.

- `headless:=true rviz:=false`: 창 없이 EGL 기반 센서 렌더링으로 서버 실행.
- `render_backend:=software`: 이 PC에서 검증한 LiDAR 렌더링 우회 설정(기본값). 시뮬레이션 속도가 느려질 수 있다. 다른 PC의 정상 그래픽 경로를 사용할 때는 `render_backend:=system`으로 비교할 수 있다.
- `run_gazebo_guarded.sh`: 실행 전체의 메모리를 RAM 6GiB·swap 512MiB로 제한한다. 벤치 및 영상 실행 스크립트에도 적용된다.
- `slam_mode:=sync`: 동기식 SLAM. 기본 `async`는 큐 크기 1로 최신 스캔 위주 처리.
- `scenario:=/절대경로/scenario.yaml`: 시작 위치·방향과 마커 변경.
- `duration:=20`: 종료·저장 확인용 짧은 실행. 기본 600초는 시뮬레이션 시간 기준.
- `ros2 service call /mission/stop std_srvs/srv/Trigger '{}'`: 조기 종료 및 저장.
- 실행별 Gazebo transport partition을 분리한다. ROS domain은 위와 같이 직접 맞춘다.

## 저장·재사용

종료 후에도 launch를 유지하여 `mission/status`가 `done`이 될 때까지 기다린다. `results/날짜_시간/`에 저장한다.

| 파일 | 내용 |
|---|---|
| map.pgm, map.yaml | 점유 격자 및 메타데이터 |
| map.posegraph, map.data | SLAM graph·스캔 데이터, 이어서 지도 작성에 필요 |
| markers.csv | id,x,y — map frame, 미터 |
| result.json | 종료 사유, 시간, 발견 수, 파일별 저장 성공 여부 |
| map_saver.log | 지도 이미지 저장 로그 |

YAML은 image, resolution(기본 0.05m), origin, negate, occupied_thresh(0.65), free_thresh(0.196)를 포함한다. YAML origin은 이미지 좌하단의 map 좌표이며 로봇 시작점과 다를 수 있다.

launch 종료 후 기존 graph로 다시 시작:

```bash
ros2 launch osp_mission simulation.launch.py session:=continue \
  posegraph:=/home/peter/simul/results/실행폴더/map resume_pose:=0,0,0
```

`posegraph`는 확장자 없는 절대 경로다. `resume_pose`는 기존 map에서 현재 로봇의 x,y,yaw이며 yaw 단위는 rad다. `session:=localization`은 기존 graph 기반 위치 추정만 한다. 새 시연은 항상 기본 `session:=new`를 사용한다. 기존 지도 재사용 시 지도 좌표계는 유지된다.

SLAM Toolbox 서비스를 직접 사용할 수도 있다:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/tmp/studio'}"
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '/tmp/studio', match_type: 2, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"
```

이는 graph를 복원해 계속 지도 작성하는 lifelong 사용 방식이다. 무기한 그래프 크기 제한·오래된 노드 자동 제거 기능은 추가하지 않았다.

## 도면과 시나리오

`src/osp_mission/config/scenario.yaml`을 복사하여 수정한다. ID는 Original ArUco 범위 0–1023, size는 검은 마커 정사각형 변 길이(m)다. yaw는 마커 앞면 법선 방향이다. `randomize: true`와 seed로 벽의 후보 위치에서 서로 다른 마커 3개를 생성한다. 알고리즘에는 scenario나 ground_truth를 전달하지 않는다. 생성한 world와 평가용 ground_truth는 `/tmp/osp_world_*`에 있다.

공간은 20.15 × 11.35m, 트랙 폭은 약 0.845m다. 중앙 빗금과 차선은 충돌 없는 바닥 표시이고 중앙도 주행 가능하다. 벽 돌출부 위치·깊이와 곡선 형상은 래스터 도면에서 추정한 근사치다. CAD 수준의 정확한 재현은 원본 DXF나 실측이 필요하다. 오른쪽 작은 그림은 상세도이므로 중복 공간으로 생성하지 않는다.

## 실제 TurtleBot4

1. 로봇 위 노트북과 로봇을 LAN으로 연결하고 서로 통신되는 IP를 설정한다. 로봇의 ROS 배포판·RMW·ROS_DOMAIN_ID·discovery 설정을 맞추고 `ros2 topic list`로 확인한다. 노트북과 로봇 시간도 동기화한다.
2. 로봇 기본 bringup을 실행한다. 기존 SLAM/Nav2가 실행 중이면 중복 실행하지 않는다. 노트북에서는 아래 공통 노드만 실행한다.
3. `ros2 topic info -v /cmd_vel`, `ros2 topic list`, `ros2 run tf2_ros tf2_echo odom base_link`로 타입과 TF를 확인한다. 이 프로젝트는 공식 Jazzy 설정에 맞춰 TwistStamped를 사용한다.

```bash
ros2 launch osp_mission robot.launch.py \
  rgb_topic:=oakd/rgb/preview/image_raw \
  camera_info_topic:=oakd/rgb/preview/camera_info \
  depth_topic:=unused_depth
```

기본 실제 로봇 사용은 **LiDAR 부착면 기반 거리 추정**으로 한다. 깊이를 사용하려면 RGB와 같은 해상도·시야·좌표계로 정렬된 depth 토픽을 지정한다. 원시 stereo depth를 RGB depth처럼 쓰면 안 된다. `scan_topic`, `base_frame`, `namespace`도 실제 bringup에 맞춰 지정할 수 있다. RViz는 기본 무네임스페이스 토픽을 사용하므로 namespace를 쓰면 Displays의 토픽을 해당 prefix로 변경한다.

실제 로봇은 기본 `undock:=false`이며 출발 지점에 놓고 시작한다. 도크에서 시작할 경우 `undock:=true`를 지정한다. 센서나 TF가 없으면 정지하고 Nav2 목표를 취소한다. 복구되면 탐색을 재개한다. 제한시간은 실제 로봇에서는 단조 증가 시간이다.

## 검증 및 한계

```bash
source install/setup.bash
python3 -m pytest src/osp_mission/test -q
```

단위 검증은 도달 가능한 frontier, 중앙 표시의 충돌 제외, 마커 패턴의 ID, 무작위 배치 재현성, 크기가 다른 마커의 깊이 기반 좌표 및 관측 확정을 확인한다. 실제 10분 내 발견률과 0.3m 위치 오차는 성능 목표이며 보장값이 아니다.

마커 관측은 odom에 누적하고 최신 map→odom 보정으로 표시한다. 큰 loop closure의 비강체 그래프 수정에 대한 개별 관측 재최적화는 구현하지 않았으므로, 장거리 누적 오차가 크면 재관측이 필요하다. 자동 저장한 마커 목록은 해당 실행에서 발견한 목록이며 이전 실행 목록은 자동 병합하지 않는다.

## 초기 구현 단계에서 확인한 결과

- colcon build 성공, pytest 4개 통과.
- LiDAR 640개 샘플 중 유효 거리 약 1–12m 수신 및 점유·자유 격자 생성 확인.
- 시작 서비스, undock에 따른 odom 이동, 시간 초과 시 정지 및 5개 결과 파일 저장 확인.
- Nav2 NavigateToPose 주행 요청이 SUCCEEDED(status 4, error_code 0)로 완료됨.
- 저장한 pose graph를 Sync 노드에서 읽고 Nav2 활성화 확인.
- 마커 7 앞에서 실제 렌더링 영상의 ID 7 검출 및 map 좌표 약 (1.048, 0.002)m 확인. 해당 시작 자세에서 정답은 (1.05, 0)m.
- 서로 다른 세 배치의 600초 완주 성공률과 실물 로봇 검증은 아직 수행하지 않았다.

시뮬레이션용 로봇 description은 설치된 모델을 메모리에서 생성한 뒤 센서 엔진만 Ogre2로 통일한다. `/opt/ros` 파일은 수정하지 않는다. 마커는 흑백 셀을 실제 visual geometry로 생성하여 텍스처 로딩 의존성을 피한다.

## 장애물 배치와 학습

다중 장애물 시나리오와 CEM 탐색 정책의 현재 실행·검증은 [TRAINING.md](TRAINING.md)를 참고한다. 초기 숨김 과제의 14.6% 시험 결과와 연속 감독은 역사적 기록이다. 현재 강화 배치는 별도 학습으로 진행한다.

현재 기본 시나리오는 강화 배치 `config/robust_demo.yaml`이다. 장애물 12개와 서로 다른 장애물의 시작점 반대 면에 붙인 Original ArUco 3개를 배치했다. PDF 예시 기반의 쉬운 4개 배치는 `scenario:=.../config/assignment_demo.yaml`로 재사용할 수 있다. 기본 카메라는 960×720·5fps이며, 마커 위치는 탐색기에 전달하지 않는다. 이전의 장애물 뒤 숨김 과제는 별도 고난도 실험으로 보존했다. 최신 조건·평가와 실행법은 [TRAINING.md](TRAINING.md)의 맨 위 절을 참고한다.

학습은 시연 장면 하나를 반복하지 않는다. 이전 학습은 200개 배치를 모두 사용했고, 새 v2 풀은 학습 400개·검증 160개다. 새 학습 두 건은 각각 이 중 160개 배치까지 사용했으며 전체 400개를 모두 학습한 것은 아니다. 유형별 중복 없는 추출과 지도 좌표 회전을 적용한다. `planner:=directional`과 `planner:=edge`는 부착면의 카메라 관측을 구분하는 비교용 모드다. `edge`는 얇은 장애물의 앞뒤 면을 별도 상태로 기록한다. 새 시험용 시드 200개는 아직 생성·평가하지 않았다.

시뮬레이션의 현재 기본값은 `planner:=surface_endpoint`와 `config/surface_endpoint_policy.json`이다. 실제로 처리된 RGB와 신선한 LiDAR·TF가 있어야 표면 관측을 누적한다. 이전 정책은 `planner:=legacy`로 비교할 수 있다. 도크 이탈 취소 문제는 수정했고, 현재는 LiDAR 렌더링과 IMU 기반 회전 추정을 수정한 환경에서 탐색 성능과 마커 좌표 정확도를 검증하고 있다.

실제 Gazebo 관찰 화면·로봇 카메라·LiDAR와 SLAM 지도 확장을 함께 녹화하려면 호스트 터미널에서 `bash /home/peter/simul/run_gazebo_video.sh`를 실행한다. 장애물 12개와 장애물 뒷면 마커 3개 배치이며, 출력 폴더의 `video/mission.mp4`에 저장한다. 같은 실패 배치 재검사는 `bash /home/peter/simul/run_gazebo_runtime_probe.sh --record-video`다. 자세한 검증 범위와 결과 위치는 [TRAINING.md](TRAINING.md)의 첫 절에 있다.

## GitHub 유지보수 및 영상 추가

작업 완료 시 관련 검증 → 변경 검토 → 커밋 → `main` 푸시를 수행합니다.
이 지침은 [AGENTS.md](AGENTS.md)에 저장했습니다. 별도의 상시 자동 코드 배포 작업은 설정하지 않았습니다.

완료된 Gazebo 영상은 다음 명령으로 Release에 추가합니다. 이미 업로드한 파일은 SHA-256을
대조하고 건너뛰며, 최종 평가나 정상적으로 마감된 영상이 없는 실행은 보류합니다.

```bash
python3 scripts/publish_simulation_videos.py --upload
```

명령 실행 후 갱신된 `repository_snapshot/videos.json`과 영상 목록을 검토하고 커밋합니다.
전체 실험 영상은 Git 이력에 넣지 않고 Release 자산으로 관리합니다.

<!-- SIMULATION_VIDEO_INDEX:START -->

## 저장된 Gazebo 영상

[GitHub Release에서 다운로드](https://github.com/peter010905/simul/releases/tag/simulation-videos-2026-11) · 검증된 영상 88개

아래 성공 여부는 당시 평가 결과이며, 현재 기준의 독립 완주율을 뜻하지 않습니다.

<details><summary>전체 영상과 평가 결과 펼치기</summary>

| 영상 | 당시 평가 | 실제 1.5m 접근 | 시간(시뮬레이션 초) |
|---|---|---|---:|
| [fix_v10_escape_regression3__9802005.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v10_escape_regression3__9802005.mp4) | 성공 | 확인 | 335.45 |
| [fix_v10_escape_regression3__9802009.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v10_escape_regression3__9802009.mp4) | 성공 | 확인 | 345.245 |
| [fix_v11_completion_regression1__9802000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v11_completion_regression1__9802000.mp4) | 성공 | 확인 | 182.5 |
| [fix_v12_identity_regression1__9802012.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v12_identity_regression1__9802012.mp4) | 성공 | 확인 | 55.195 |
| [approach_rpp_v8_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_rpp_v8_screen3__8600007.mp4) | 성공 | 확인 | 386.903 |
| [approach_rpp_v8_screen3__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_rpp_v8_screen3__8600133.mp4) | 실패 | 확인 | 600.0 |
| [approach_rpp_v8_screen3__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_rpp_v8_screen3__8600139.mp4) | 실패 | 확인 | 600.3 |
| [approach_v8_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_v8_screen3__8600007.mp4) | 성공 | 확인 | 512.519 |
| [approach_v9_io_remaining2__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_v9_io_remaining2__8600133.mp4) | 실패 | 확인 | 600.467 |
| [approach_v9_io_remaining2__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_v9_io_remaining2__8600139.mp4) | 성공 | 확인 | 524.0 |
| [development_frontier_v1__runtime_probes__20261104T225516_124417Z.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_frontier_v1__runtime_probes__20261104T225516_124417Z.mp4) | 실패 | 미달 | 136.514 |
| [development_imu_screen4__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600007.mp4) | 실패 | 미달 | 600.049 |
| [development_imu_screen4__8600040.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600040.mp4) | 성공 | 미달 | 117.2 |
| [development_imu_screen4__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600133.mp4) | 실패 | 미달 | 600.0 |
| [development_imu_screen4__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600139.mp4) | 실패 | 미달 | 600.1 |
| [development_imu_v1__runtime_probes__20261104T230250_371315Z.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_v1__runtime_probes__20261104T230250_371315Z.mp4) | 성공 | 미달 | 238.9 |
| [edge_runtime_v2__runtime_probes__20261104T234253_265975Z.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/edge_runtime_v2__runtime_probes__20261104T234253_265975Z.mp4) | 실패 | 미달 | 600.306 |
| [marker_focus_v6_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_focus_v6_screen3__8600007.mp4) | 실패 | 자료 없음 | 242.819 |
| [marker_focus_v6_service_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_focus_v6_service_screen3__8600007.mp4) | 실패 | 미달 | 347.089 |
| [motion_v1_mixed16__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/motion_v1_mixed16__8600007.mp4) | 실패 | 미달 | 68.797 |
| [motion_v2_drive_ready_mixed16__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/motion_v2_drive_ready_mixed16__8600007.mp4) | 성공 | 확인 | 356.447 |
| [native_search_v1__001_screen_mppi_frontier_9801000__9801000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__001_screen_mppi_frontier_9801000__9801000.mp4) | 성공 | 확인 | 303.6 |
| [native_search_v1__002_screen_mppi_short_travel_9801000__9801000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__002_screen_mppi_short_travel_9801000__9801000.mp4) | 성공 | 확인 | 408.815 |
| [native_search_v1__003_screen_rpp_balanced_9801000__9801000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__003_screen_rpp_balanced_9801000__9801000.mp4) | 성공 | 확인 | 552.611 |
| [native_search_v1__004_screen_mppi_balanced_9801001__9801001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__004_screen_mppi_balanced_9801001__9801001.mp4) | 성공 | 확인 | 285.139 |
| [native_search_v1__005_screen_mppi_frontier_9801001__9801001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__005_screen_mppi_frontier_9801001__9801001.mp4) | 성공 | 확인 | 559.059 |
| [native_search_v1__006_screen_mppi_short_travel_9801001__9801001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__006_screen_mppi_short_travel_9801001__9801001.mp4) | 성공 | 확인 | 331.753 |
| [native_search_v1__007_screen_rpp_balanced_9801001__9801001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__007_screen_rpp_balanced_9801001__9801001.mp4) | 성공 | 확인 | 268.557 |
| [native_search_v1__008_screen_mppi_balanced_9801002__9801002.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__008_screen_mppi_balanced_9801002__9801002.mp4) | 성공 | 확인 | 307.203 |
| [native_search_v1__009_screen_mppi_frontier_9801002__9801002.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__009_screen_mppi_frontier_9801002__9801002.mp4) | 성공 | 확인 | 449.9 |
| [native_search_v1__010_screen_mppi_short_travel_9801002__9801002.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__010_screen_mppi_short_travel_9801002__9801002.mp4) | 성공 | 확인 | 423.604 |
| [native_search_v1__011_screen_rpp_balanced_9801002__9801002.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__011_screen_rpp_balanced_9801002__9801002.mp4) | 성공 | 확인 | 456.284 |
| [native_search_v1__012_screen_mppi_balanced_9801003__9801003.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__012_screen_mppi_balanced_9801003__9801003.mp4) | 실패 | 확인 | 600.0 |
| [native_search_v1__013_screen_mppi_frontier_9801003__9801003.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__013_screen_mppi_frontier_9801003__9801003.mp4) | 성공 | 확인 | 419.7 |
| [native_search_v1__014_screen_mppi_short_travel_9801003__9801003.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__014_screen_mppi_short_travel_9801003__9801003.mp4) | 실패 | 확인 | 600.0 |
| [native_search_v1__015_screen_rpp_balanced_9801003__9801003.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__015_screen_rpp_balanced_9801003__9801003.mp4) | 성공 | 확인 | 489.978 |
| [native_search_v1__016_screen_mppi_balanced_9801004__9801004.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__016_screen_mppi_balanced_9801004__9801004.mp4) | 실패 | 미달 | 600.389 |
| [native_search_v1__017_screen_mppi_frontier_9801004__9801004.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__017_screen_mppi_frontier_9801004__9801004.mp4) | 성공 | 확인 | 572.862 |
| [native_search_v1__018_screen_mppi_short_travel_9801004__9801004.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__018_screen_mppi_short_travel_9801004__9801004.mp4) | 실패 | 미달 | 600.3 |
| [native_search_v1__019_screen_rpp_balanced_9801004__9801004.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__019_screen_rpp_balanced_9801004__9801004.mp4) | 성공 | 확인 | 515.6 |
| [native_search_v1__020_validation_rpp_balanced_9802000__9802000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__020_validation_rpp_balanced_9802000__9802000.mp4) | 성공 | 확인 | 472.637 |
| [native_search_v1__021_validation_mppi_frontier_9802000__9802000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__021_validation_mppi_frontier_9802000__9802000.mp4) | 실패 | 확인 | 600.0 |
| [native_search_v1__022_validation_rpp_balanced_9802001__9802001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__022_validation_rpp_balanced_9802001__9802001.mp4) | 실패 | 미달 | 600.289 |
| [native_search_v1__023_validation_mppi_frontier_9802001__9802001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__023_validation_mppi_frontier_9802001__9802001.mp4) | 실패 | 미달 | 600.118 |
| [native_search_v1__024_validation_rpp_balanced_9802002__9802002.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__024_validation_rpp_balanced_9802002__9802002.mp4) | 성공 | 확인 | 546.3 |
| [native_search_v1__025_validation_mppi_frontier_9802002__9802002.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__025_validation_mppi_frontier_9802002__9802002.mp4) | 실패 | 확인 | 600.245 |
| [native_search_v1__026_validation_rpp_balanced_9802003__9802003.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__026_validation_rpp_balanced_9802003__9802003.mp4) | 성공 | 확인 | 453.6 |
| [native_search_v1__027_validation_mppi_frontier_9802003__9802003.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__027_validation_mppi_frontier_9802003__9802003.mp4) | 성공 | 확인 | 431.25 |
| [native_search_v1__028_validation_rpp_balanced_9802004__9802004.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__028_validation_rpp_balanced_9802004__9802004.mp4) | 실패 | 확인 | 600.103 |
| [native_search_v1__029_validation_mppi_frontier_9802004__9802004.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__029_validation_mppi_frontier_9802004__9802004.mp4) | 성공 | 확인 | 579.1 |
| [native_search_v1__030_validation_rpp_balanced_9802005__9802005.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__030_validation_rpp_balanced_9802005__9802005.mp4) | 실패 | 확인 | 600.042 |
| [native_search_v1__031_validation_mppi_frontier_9802005__9802005.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__031_validation_mppi_frontier_9802005__9802005.mp4) | 실패 | 미달 | 600.146 |
| [native_search_v1__032_validation_rpp_balanced_9802006__9802006.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__032_validation_rpp_balanced_9802006__9802006.mp4) | 성공 | 확인 | 369.635 |
| [native_search_v1__033_validation_mppi_frontier_9802006__9802006.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__033_validation_mppi_frontier_9802006__9802006.mp4) | 성공 | 확인 | 458.061 |
| [native_search_v1__034_validation_rpp_balanced_9802007__9802007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__034_validation_rpp_balanced_9802007__9802007.mp4) | 성공 | 확인 | 316.583 |
| [native_search_v1__035_validation_mppi_frontier_9802007__9802007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__035_validation_mppi_frontier_9802007__9802007.mp4) | 실패 | 미달 | 600.4 |
| [native_search_v1__036_validation_rpp_balanced_9802008__9802008.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__036_validation_rpp_balanced_9802008__9802008.mp4) | 성공 | 확인 | 464.379 |
| [native_search_v1__037_validation_mppi_frontier_9802008__9802008.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__037_validation_mppi_frontier_9802008__9802008.mp4) | 성공 | 확인 | 237.3 |
| [native_search_v1__038_validation_rpp_balanced_9802009__9802009.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__038_validation_rpp_balanced_9802009__9802009.mp4) | 실패 | 미달 | 600.0 |
| [native_search_v1__039_validation_mppi_frontier_9802009__9802009.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__039_validation_mppi_frontier_9802009__9802009.mp4) | 실패 | 미달 | 600.443 |
| [native_search_v1__040_validation_rpp_balanced_9802010__9802010.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__040_validation_rpp_balanced_9802010__9802010.mp4) | 성공 | 확인 | 585.741 |
| [native_search_v1__041_validation_mppi_frontier_9802010__9802010.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__041_validation_mppi_frontier_9802010__9802010.mp4) | 성공 | 확인 | 492.0 |
| [native_search_v1__042_validation_rpp_balanced_9802011__9802011.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__042_validation_rpp_balanced_9802011__9802011.mp4) | 실패 | 확인 | 600.012 |
| [native_search_v1__043_validation_mppi_frontier_9802011__9802011.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__043_validation_mppi_frontier_9802011__9802011.mp4) | 성공 | 확인 | 347.916 |
| [native_search_v1__044_validation_rpp_balanced_9802012__9802012.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__044_validation_rpp_balanced_9802012__9802012.mp4) | 실패 | 확인 | 600.2 |
| [native_search_v1__045_validation_mppi_frontier_9802012__9802012.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__045_validation_mppi_frontier_9802012__9802012.mp4) | 성공 | 확인 | 118.532 |
| [native_search_v1__046_validation_rpp_balanced_9802013__9802013.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__046_validation_rpp_balanced_9802013__9802013.mp4) | 성공 | 확인 | 510.104 |
| [native_search_v1__047_validation_mppi_frontier_9802013__9802013.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__047_validation_mppi_frontier_9802013__9802013.mp4) | 실패 | 미달 | 600.365 |
| [native_search_v1__048_validation_rpp_balanced_9802014__9802014.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__048_validation_rpp_balanced_9802014__9802014.mp4) | 실패 | 미달 | 600.254 |
| [native_search_v1__049_validation_mppi_frontier_9802014__9802014.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__049_validation_mppi_frontier_9802014__9802014.mp4) | 실패 | 확인 | 600.213 |
| [native_search_v1__050_validation_rpp_balanced_9802015__9802015.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__050_validation_rpp_balanced_9802015__9802015.mp4) | 실패 | 확인 | 600.339 |
| [native_search_v1__051_validation_mppi_frontier_9802015__9802015.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__051_validation_mppi_frontier_9802015__9802015.mp4) | 실패 | 확인 | 600.176 |
| [native_search_v1__052_validation_rpp_balanced_9802016__9802016.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__052_validation_rpp_balanced_9802016__9802016.mp4) | 실패 | 미달 | 600.0 |
| [native_search_v1__053_validation_mppi_frontier_9802016__9802016.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__053_validation_mppi_frontier_9802016__9802016.mp4) | 성공 | 확인 | 580.672 |
| [native_search_v1__054_validation_rpp_balanced_9802017__9802017.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__054_validation_rpp_balanced_9802017__9802017.mp4) | 실패 | 확인 | 600.106 |
| [native_search_v1__055_validation_mppi_frontier_9802017__9802017.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__055_validation_mppi_frontier_9802017__9802017.mp4) | 실패 | 확인 | 600.09 |
| [native_search_v1__056_validation_rpp_balanced_9802018__9802018.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__056_validation_rpp_balanced_9802018__9802018.mp4) | 실패 | 미달 | 600.477 |
| [native_search_v1__057_validation_mppi_frontier_9802018__9802018.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__057_validation_mppi_frontier_9802018__9802018.mp4) | 실패 | 미달 | 600.307 |
| [native_search_v1__058_validation_rpp_balanced_9802019__9802019.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__058_validation_rpp_balanced_9802019__9802019.mp4) | 성공 | 확인 | 546.723 |
| [native_search_v1__059_validation_mppi_frontier_9802019__9802019.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/native_search_v1__059_validation_mppi_frontier_9802019__9802019.mp4) | 성공 | 확인 | 522.737 |
| [panorama_v5_screen2__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/panorama_v5_screen2__8600007.mp4) | 실패 | 미달 | 600.0 |
| [panorama_v5_screen2__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/panorama_v5_screen2__8600139.mp4) | 성공 | 미달 | 293.6 |
| [persistent_edge_v3_mixed16__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_edge_v3_mixed16__8600007.mp4) | 실패 | 미달 | 358.113 |
| [persistent_map_v4_recovery32__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_recovery32__8600139.mp4) | 실패 | 미달 | 600.464 |
| [persistent_map_v4_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_screen3__8600007.mp4) | 실패 | 미달 | 600.12 |
| [persistent_map_v4_screen3__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_screen3__8600133.mp4) | 실패 | 미달 | 600.305 |
| [persistent_map_v4_screen3__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_screen3__8600139.mp4) | 실패 | 자료 없음 | 257.887 |
| [rpp_v7_service_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/rpp_v7_service_screen3__8600007.mp4) | 실패 | 미달 | 347.66799999999995 |

</details>

<!-- SIMULATION_VIDEO_INDEX:END -->
