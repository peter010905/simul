# OSP Final Project — ROS 2 Jazzy

> **2026-11-13 현재:** 원형·삼각형·사다리꼴·직사각형·L자·U자·다중 방의 외곽과 주행 공간을 바꾸며 실제 Gazebo에서 학습합니다.
> 수정한 센서·위치 추정·탐색 소스를 고정한 `spatial_cem_v6`가 시작됐습니다. 첫28개 훈련 맵의 외곽선과 점유 격자는 모두 다르며,
> 장애물8–32개, 출발 자세, 마커 크기와 벽·장애물 부착 위치도 바뀝니다. 매 훈련 주행마다 새 맵을 사용합니다.
> CEM으로 SLAM/Nav2 탐색 계수를 조정하며, 독립 일반화 시험은 아직 완료하지 않았습니다. 실사용 정책으로 승격하지 않았습니다.
> [현재 학습·기준·실행 상태](repository_snapshot/spatial_curriculum.json) · [28개 공간 중복·접근 가능성 검사](repository_snapshot/spatial_geometry_v6.json) · [구성한 공간 예시](repository_snapshot/spatial_training_v6_preview.png).
>
> **최신 실제 비교:** 원형의 같은 맵·가중치에서 근거리 관측 유지 방식을 고친 후보가 **236.793초**에 완주했습니다. 기준은399.2초였습니다.
> 세 ID, 각 실제1.5m 접근, 지도 좌표 오차0.5m 이내, 충돌0회와 저장된 근거리 관측9개의 실제 접근 시점을 모두 확인했습니다.
> 지도 좌표 오차는 최대13.34cm였습니다. 이번1회씩의 비교에서는40.68% 짧았으며, 여러 공간에서 같은 개선 폭이 나온다는 뜻은 아닙니다.
> 원통 마커의 반복 접근 요청은4회→1회였지만, 전체 재통과 거리 비율은6.22%→9.47%였습니다. 재통과 거리 자체는 약4m로 비슷해 전체 재방문 감소를 주장하지 않습니다.
> [비교 결과·물리적 접근 증거·영상](repository_snapshot/windowed_witness_candidate.json).
>
> **남은 실패:** 이전 결합 후보의 새 삼각형은360.417초에 전체 기준을 통과했습니다. 사다리꼴은580.702초에 세 ID와 실제 접근·충돌0회를 달성했지만,
> 한 마커의 지도 좌표 오차1.215m로 실패했습니다. 실제 자세를 쓴 평가용 재생에서는 해당 관측의 오차 중앙값이1.215m→5.37mm여서 잔여 위치 추정 오차를 지지합니다.
> 이 실제 자세 보정은 분석에만 쓰며 정책에 전달하지 않습니다. [공간별 실제 결과](repository_snapshot/spatial_calibrated_screen.json) · [사다리꼴 실패 분석과 한계](repository_snapshot/calibrated_trapezoid_localization_failure.json).
>
> **평가와 재방문:** 정확한3개 ID·각 실제1.5m 접근·좌표 오차0.5m 이내·충돌0회·600시뮬레이션 초 이내를 유지합니다.
> 확인된 ID의 몸체 기준1.3m 이내 정상 센서 관측3개를1초 안에서 모으며, 누락된 라이다 프레임은 접근 근거를 추가하지 않습니다.
> odom 기준 관측 기억·반복 목표 억제와 실제 경로·목표 재방문 감사를 기록합니다. 필요한 통로 왕복도 재통과 거리에 포함됩니다.
> 모든 계수 후보는 같은 수정 소스를 사용합니다. 이후 같은 맵의 보정된 이전 정책과7쌍 파일럿·28쌍 검증 후 새 공간42회와 원래 과제40회 기준을 적용합니다.
> 실제 Gazebo는 최대 두 주행, 각6GiB 제한으로 실행하며 진행 중이던 이전 주행은 종료할 때까지 보존했습니다.
> [이전 학습 원본과 전환 상태](repository_snapshot/spatial_curriculum_v5_at_transition.json).
>
> **확인 방법:** [로컬 로봇 대시보드](http://127.0.0.1:8765)에서 RGB 카메라·SLAM·LiDAR·ID/접근 로그와 완료 영상을 확인합니다.
> 실행: `bash scripts/run_robot_dashboard.sh start`. [대시보드 사용법](dashboard/README.md).
> 새 학습은 마커 중심의 깊이값 유효 개수·중앙값·편차도 읽기 전용으로 기록합니다. 이 기록만으로 검출기가 실제 사용한 거리 센서를 단정하지 않습니다.
> 시뮬레이션 바퀴 속도 보정0.78은 실제 로봇에 검증된 값이 아닙니다. 하드웨어 검증과 모든 공간 일반화는 아직 미완료입니다.

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

현재 v13 후보의 탐색 코드·edge 가중치·RPP 설정을 함께 적용하려면 저장소 루트의
`run_robot_policy.sh`를 사용합니다. 로봇의 기본 센서/베이스 bringup을 먼저 실행합니다.
이 스크립트가 SLAM과 Nav2를 시작하므로 동일 namespace에 이미 있으면 점검 단계에서 종료합니다.
아래 `ROBOT_DOMAIN`을 로봇에 설정된 ROS_DOMAIN_ID 숫자로 바꿉니다.

```bash
bash /home/peter/simul/run_robot_policy.sh --domain ROBOT_DOMAIN --build-only
bash /home/peter/simul/run_robot_policy.sh --domain ROBOT_DOMAIN --check
bash /home/peter/simul/run_robot_policy.sh --domain ROBOT_DOMAIN --start --rviz
```

`--start`를 생략하면 임무는 idle 상태이며 출력된 `mission/start` 서비스로 시작합니다.
`mission/stop` 서비스로 정지할 수 있습니다. `--namespace`, `--rgb`, `--camera-info`, `--scan`,
`--duration`, `--undock`을 지원하며 `--help`에 기본값을 표시합니다.
RGB와 CameraInfo는 같은 영상에 해당해야 하며, 노트북과 로봇의 RMW·discovery 설정과 시각을 맞춥니다.
이 실행기는 Jazzy의 `TwistStamped` 인터페이스를 사용합니다.

배포용 코드는 `deployment/robot_v13`에 복사·빌드합니다.
접근/마커 후속 관측의 센서 시각 비교를 ROS 시간으로 맞췄고, 임무 제한시간에는 단조 증가 시간을 유지합니다.
실행 결과는 `results/robot_*`에 저장합니다. 다른 PC에서는 저장소를 복제하고 해당 경로의 스크립트를 실행합니다.
v13은 개발 후보이며 실물 검증은 아직입니다. 앞서 기록한 작은 마커 관측 기준, 지도 관측 기억 손실,
ID 충돌 해제와 접근 상태 표시 문제는 남아 있습니다.

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

[GitHub Release에서 다운로드](https://github.com/peter010905/simul/releases/tag/simulation-videos-2026-11) · 검증된 영상 171개

아래 성공 여부는 당시 평가 결과이며, 현재 기준의 독립 완주율을 뜻하지 않습니다. 환경 오류로 종료한 영상은 정책 학습 표본에서 제외합니다.

<details><summary>전체 영상과 평가 결과 펼치기</summary>

| 영상 | 당시 평가 | 실제 1.5m 접근 | 시간(시뮬레이션 초) |
|---|---|---|---:|
| [fix_v10_escape_regression3__9802005.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v10_escape_regression3__9802005.mp4) | 성공 | 확인 | 335.45 |
| [fix_v10_escape_regression3__9802009.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v10_escape_regression3__9802009.mp4) | 성공 | 확인 | 345.245 |
| [fix_v10_escape_regression3__9802018.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v10_escape_regression3__9802018.mp4) | 실패 | 확인 | 600.477 |
| [fix_v11_completion_regression1__9802000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v11_completion_regression1__9802000.mp4) | 성공 | 확인 | 182.5 |
| [fix_v12_identity_regression1__9802012.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v12_identity_regression1__9802012.mp4) | 성공 | 확인 | 55.195 |
| [fix_v13_observation_regression6__9801004.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v13_observation_regression6__9801004.mp4) | 성공 | 확인 | 555.6 |
| [fix_v13_observation_regression6__9802001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v13_observation_regression6__9802001.mp4) | 성공 | 확인 | 210.411 |
| [fix_v13_observation_regression6__9802011.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v13_observation_regression6__9802011.mp4) | 성공 | 확인 | 218.153 |
| [fix_v13_observation_regression6__9802014.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v13_observation_regression6__9802014.mp4) | 실패 | 미달 | 600.416 |
| [fix_v13_observation_regression6__9802015.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v13_observation_regression6__9802015.mp4) | 실패 | 미달 | 600.269 |
| [fix_v13_observation_regression6__9802017.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fix_v13_observation_regression6__9802017.mp4) | 실패 | 미달 | 600.411 |
| [approach_rpp_v8_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_rpp_v8_screen3__8600007.mp4) | 성공 | 확인 | 386.903 |
| [approach_rpp_v8_screen3__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_rpp_v8_screen3__8600133.mp4) | 실패 | 확인 | 600.0 |
| [approach_rpp_v8_screen3__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_rpp_v8_screen3__8600139.mp4) | 실패 | 확인 | 600.3 |
| [approach_v8_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_v8_screen3__8600007.mp4) | 성공 | 확인 | 512.519 |
| [approach_v9_io_remaining2__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_v9_io_remaining2__8600133.mp4) | 실패 | 확인 | 600.467 |
| [approach_v9_io_remaining2__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/approach_v9_io_remaining2__8600139.mp4) | 성공 | 확인 | 524.0 |
| [bounded_gain_fresh_probe_v1__case_0__1160000404.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/bounded_gain_fresh_probe_v1__case_0__1160000404.mp4) | 성공 | 확인 | 197.4 |
| [development_frontier_v1__runtime_probes__20261104T225516_124417Z.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_frontier_v1__runtime_probes__20261104T225516_124417Z.mp4) | 실패 | 미달 | 136.514 |
| [development_imu_screen4__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600007.mp4) | 실패 | 미달 | 600.049 |
| [development_imu_screen4__8600040.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600040.mp4) | 성공 | 미달 | 117.2 |
| [development_imu_screen4__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600133.mp4) | 실패 | 미달 | 600.0 |
| [development_imu_screen4__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_screen4__8600139.mp4) | 실패 | 미달 | 600.1 |
| [development_imu_v1__runtime_probes__20261104T230250_371315Z.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/development_imu_v1__runtime_probes__20261104T230250_371315Z.mp4) | 성공 | 미달 | 238.9 |
| [edge_runtime_v2__runtime_probes__20261104T234253_265975Z.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/edge_runtime_v2__runtime_probes__20261104T234253_265975Z.mp4) | 실패 | 미달 | 600.306 |
| [floor_pose_regression_v1__case_0__1136000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/floor_pose_regression_v1__case_0__1136000000.mp4) | 성공 | 확인 | 288.903 |
| [floor_pose_regression_v1__case_1__1136000101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/floor_pose_regression_v1__case_1__1136000101.mp4) | 성공 | 확인 | 427.501 |
| [fresh_map_cem_v1__00000_train_g0_0__1121200000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v1__00000_train_g0_0__1121200000.mp4) | 실패 | 미달 | 600.455 |
| [fresh_map_cem_v1__00001_train_g0_1__1121200101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v1__00001_train_g0_1__1121200101.mp4) | 실패 | 미달 | 600.309 |
| [fresh_map_cem_v1__00002_train_g0_2__1121200202.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v1__00002_train_g0_2__1121200202.mp4) | 실패 | 확인 | 600.3 |
| [fresh_map_cem_v1__00003_train_g0_3__1121200303.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v1__00003_train_g0_3__1121200303.mp4) | 성공 | 확인 | 317.4 |
| [fresh_map_cem_v1__00004_train_g0_0__1121200404.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v1__00004_train_g0_0__1121200404.mp4) | 성공 | 확인 | 362.907 |
| [fresh_map_cem_v1__00005_train_g0_1__1121200505.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v1__00005_train_g0_1__1121200505.mp4) | 성공 | 확인 | 318.301 |
| [fresh_map_cem_v2__00000_train_g0_0__1123200000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v2__00000_train_g0_0__1123200000.mp4) | 성공 | 확인 | 581.66 |
| [fresh_map_cem_v2__00001_train_g0_1__1123200101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v2__00001_train_g0_1__1123200101.mp4) | 성공 | 확인 | 325.378 |
| [fresh_map_cem_v2__00002_train_g0_2__1123200202.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/fresh_map_cem_v2__00002_train_g0_2__1123200202.mp4) | 성공 | 확인 | 541.282 |
| [heading_prior_regression_v1__case_0__1163000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/heading_prior_regression_v1__case_0__1163000000.mp4) | 실패 | 확인 | 247.622 |
| [low_rotation_video_20261112__episode_8000001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/low_rotation_video_20261112__episode_8000001.mp4) | 성공 | 확인 | 494.534 |
| [marker_focus_v6_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_focus_v6_screen3__8600007.mp4) | 실패 | 자료 없음 | 242.819 |
| [marker_focus_v6_service_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_focus_v6_service_screen3__8600007.mp4) | 실패 | 미달 | 347.089 |
| [marker_sizes_v1_native5__9811000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_sizes_v1_native5__9811000.mp4) | 성공 | 확인 | 285.242 |
| [marker_sizes_v1_native5__9811006.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_sizes_v1_native5__9811006.mp4) | 성공 | 확인 | 370.5 |
| [marker_sizes_v1_native5__9811012.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_sizes_v1_native5__9811012.mp4) | 성공 | 확인 | 255.3 |
| [marker_sizes_v1_native5__9811018.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_sizes_v1_native5__9811018.mp4) | 실패 | 미달 | 600.383 |
| [marker_sizes_v1_native5__9811019.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/marker_sizes_v1_native5__9811019.mp4) | 실패 | 미달 | 600.261 |
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
| [navigation_regression_v1__case_0__1159000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/navigation_regression_v1__case_0__1159000000.mp4) | 성공 | 확인 | 270.5 |
| [navigation_regression_v1__case_1__1159000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/navigation_regression_v1__case_1__1159000000.mp4) | 성공 | 확인 | 504.60200000000003 |
| [panorama_v5_screen2__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/panorama_v5_screen2__8600007.mp4) | 실패 | 미달 | 600.0 |
| [panorama_v5_screen2__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/panorama_v5_screen2__8600139.mp4) | 성공 | 미달 | 293.6 |
| [persistent_edge_v3_mixed16__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_edge_v3_mixed16__8600007.mp4) | 실패 | 미달 | 358.113 |
| [persistent_map_v4_recovery32__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_recovery32__8600139.mp4) | 실패 | 미달 | 600.464 |
| [persistent_map_v4_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_screen3__8600007.mp4) | 실패 | 미달 | 600.12 |
| [persistent_map_v4_screen3__8600133.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_screen3__8600133.mp4) | 실패 | 미달 | 600.305 |
| [persistent_map_v4_screen3__8600139.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/persistent_map_v4_screen3__8600139.mp4) | 실패 | 자료 없음 | 257.887 |
| [reacquisition_startup_regression_v2__case_0__1163000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/reacquisition_startup_regression_v2__case_0__1163000000.mp4) | 실패 | 확인 | 188.674 |
| [route_v14_timecost_screen3__9802001.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/route_v14_timecost_screen3__9802001.mp4) | 성공 | 확인 | 394.932 |
| [route_v14_timecost_screen3__9802011.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/route_v14_timecost_screen3__9802011.mp4) | 성공 | 확인 | 302.6 |
| [route_v14_timecost_screen3__9802014.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/route_v14_timecost_screen3__9802014.mp4) | 실패 | 미달 | 600.151 |
| [rpp_v7_service_screen3__8600007.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/rpp_v7_service_screen3__8600007.mp4) | 실패 | 미달 | 347.66799999999995 |
| [slam_nav2_search_v1__000_screen_control_9810000__9810000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__000_screen_control_9810000__9810000.mp4) | 실패 | 미달 | 600.221 |
| [slam_nav2_search_v1__001_screen_stable_map_9810000__9810000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__001_screen_stable_map_9810000__9810000.mp4) | 실패 | 미달 | 600.151 |
| [slam_nav2_search_v1__002_screen_frontier_balanced_9810000__9810000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__002_screen_frontier_balanced_9810000__9810000.mp4) | 성공 | 확인 | 576.891 |
| [slam_nav2_search_v1__004_screen_stable_map_9810012__9810012.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__004_screen_stable_map_9810012__9810012.mp4) | 성공 | 확인 | 315.2 |
| [slam_nav2_search_v1__005_screen_frontier_balanced_9810012__9810012.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__005_screen_frontier_balanced_9810012__9810012.mp4) | 성공 | 확인 | 178.416 |
| [slam_nav2_search_v1__006_screen_control_9810018__9810018.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__006_screen_control_9810018__9810018.mp4) | 실패 | 미달 | 600.412 |
| [slam_nav2_search_v1__007_screen_stable_map_9810018__9810018.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__007_screen_stable_map_9810018__9810018.mp4) | 실패 | 미달 | 600.45 |
| [slam_nav2_search_v1__008_screen_frontier_balanced_9810018__9810018.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__008_screen_frontier_balanced_9810018__9810018.mp4) | 실패 | 미달 | 600.122 |
| [slam_nav2_search_v1__009_validation_frontier_balanced_9811006__9811006.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__009_validation_frontier_balanced_9811006__9811006.mp4) | 성공 | 확인 | 287.908 |
| [slam_nav2_search_v1__010_validation_frontier_balanced_9811013__9811013.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__010_validation_frontier_balanced_9811013__9811013.mp4) | 성공 | 확인 | 428.354 |
| [slam_nav2_search_v1__011_validation_frontier_balanced_9811019__9811019.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/slam_nav2_search_v1__011_validation_frontier_balanced_9811019__9811019.mp4) | 실패 | 확인 | 600.083 |
| [small_marker_followup_v2__regression__case_0__1121200000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/small_marker_followup_v2__regression__case_0__1121200000.mp4) | 성공 | 확인 | 282.519 |
| [small_marker_followup_v2__regression__case_1__1121200101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/small_marker_followup_v2__regression__case_1__1121200101.mp4) | 성공 | 확인 | 193.2 |
| [spatial_calibrated_screen_v1__screen_0__case_0__1171300000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_calibrated_screen_v1__screen_0__case_0__1171300000.mp4) | 성공 | 확인 | 399.2 |
| [spatial_calibrated_screen_v1__screen_1__case_0__1171300101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_calibrated_screen_v1__screen_1__case_0__1171300101.mp4) | 성공 | 확인 | 360.417 |
| [spatial_calibrated_screen_v1__screen_2__case_0__1171300202.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_calibrated_screen_v1__screen_2__case_0__1171300202.mp4) | 실패 | 확인 | 580.702 |
| [spatial_cem_v1__00000_train_g0_0__1136000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v1__00000_train_g0_0__1136000000.mp4) | 환경 오류 | 미달 | 372.574 |
| [spatial_cem_v1__00001_train_g0_0__1136000101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v1__00001_train_g0_0__1136000101.mp4) | 환경 오류 | 미달 | 375.854 |
| [spatial_cem_v2__00000_train_g0_0__1148000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v2__00000_train_g0_0__1148000000.mp4) | 성공 | 확인 | 139.266 |
| [spatial_cem_v2__00001_train_g0_0__1148000101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v2__00001_train_g0_0__1148000101.mp4) | 실패 | 미달 | 600.019 |
| [spatial_cem_v3__00000_train_g0_0__1159000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v3__00000_train_g0_0__1159000000.mp4) | 실패 | 미달 | 600.2 |
| [spatial_cem_v3__00002_train_g0_0__1159000202.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v3__00002_train_g0_0__1159000202.mp4) | 성공 | 확인 | 470.371 |
| [spatial_cem_v3__00003_train_g0_0__1159000303.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v3__00003_train_g0_0__1159000303.mp4) | 성공 | 확인 | 588.901 |
| [spatial_cem_v5__00000_train_g0_0__1163000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00000_train_g0_0__1163000000.mp4) | 실패 | 미달 | 600.222 |
| [spatial_cem_v5__00001_train_g0_0__1163000101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00001_train_g0_0__1163000101.mp4) | 성공 | 확인 | 408.346 |
| [spatial_cem_v5__00002_train_g0_0__1163000202.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00002_train_g0_0__1163000202.mp4) | 성공 | 확인 | 52.778999999999996 |
| [spatial_cem_v5__00003_train_g0_0__1163000303.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00003_train_g0_0__1163000303.mp4) | 성공 | 확인 | 319.661 |
| [spatial_cem_v5__00004_train_g0_0__1163000404.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00004_train_g0_0__1163000404.mp4) | 성공 | 확인 | 225.069 |
| [spatial_cem_v5__00005_train_g0_0__1163000505.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00005_train_g0_0__1163000505.mp4) | 성공 | 확인 | 387.504 |
| [spatial_cem_v5__00006_train_g0_0__1163000606.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00006_train_g0_0__1163000606.mp4) | 성공 | 확인 | 387.0 |
| [spatial_cem_v5__00007_train_g0_1__1163000707.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00007_train_g0_1__1163000707.mp4) | 실패 | 미달 | 600.269 |
| [spatial_cem_v5__00008_train_g0_1__1163000808.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00008_train_g0_1__1163000808.mp4) | 성공 | 확인 | 358.712 |
| [spatial_cem_v5__00009_train_g0_1__1163000909.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00009_train_g0_1__1163000909.mp4) | 성공 | 확인 | 507.1 |
| [spatial_cem_v5__00011_train_g0_1__1163001111.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00011_train_g0_1__1163001111.mp4) | 성공 | 확인 | 358.222 |
| [spatial_cem_v5__00012_train_g0_1__1163001212.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00012_train_g0_1__1163001212.mp4) | 성공 | 확인 | 59.812 |
| [spatial_cem_v5__00013_train_g0_1__1163001313.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00013_train_g0_1__1163001313.mp4) | 실패 | 확인 | 600.146 |
| [spatial_cem_v5__00014_train_g0_2__1163001414.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00014_train_g0_2__1163001414.mp4) | 성공 | 확인 | 403.502 |
| [spatial_cem_v5__00015_train_g0_2__1163001515.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00015_train_g0_2__1163001515.mp4) | 실패 | 확인 | 600.295 |
| [spatial_cem_v5__00016_train_g0_2__1163001616.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00016_train_g0_2__1163001616.mp4) | 실패 | 미달 | 600.447 |
| [spatial_cem_v5__00017_train_g0_2__1163001717.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00017_train_g0_2__1163001717.mp4) | 실패 | 확인 | 600.1 |
| [spatial_cem_v5__00018_train_g0_2__1163001818.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00018_train_g0_2__1163001818.mp4) | 실패 | 미달 | 600.4 |
| [spatial_cem_v5__00020_train_g0_2__1163002020.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00020_train_g0_2__1163002020.mp4) | 실패 | 확인 | 600.376 |
| [spatial_cem_v5__00021_train_g0_3__1163002121.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00021_train_g0_3__1163002121.mp4) | 실패 | 확인 | 265.224 |
| [spatial_cem_v5__00022_train_g0_3__1163002222.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00022_train_g0_3__1163002222.mp4) | 실패 | 확인 | 600.157 |
| [spatial_cem_v5__00023_train_g0_3__1163002323.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00023_train_g0_3__1163002323.mp4) | 실패 | 미달 | 600.236 |
| [spatial_cem_v5__00024_train_g0_3__1163002424.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/spatial_cem_v5__00024_train_g0_3__1163002424.mp4) | 실패 | 확인 | 600.2 |
| [strict_approach_regression_v1__case_0__1163000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/strict_approach_regression_v1__case_0__1163000000.mp4) | 성공 | 확인 | 496.132 |
| [tf_restamp_v1__regression__case_0__1121200000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/tf_restamp_v1__regression__case_0__1121200000.mp4) | 성공 | 확인 | 231.867 |
| [tf_restamp_v1__regression__case_1__1121200101.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/tf_restamp_v1__regression__case_1__1121200101.mp4) | 성공 | 확인 | 229.859 |
| [velocity_calibration_regression_v1__case_0__1163000000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/velocity_calibration_regression_v1__case_0__1163000000.mp4) | 성공 | 확인 | 204.2 |
| [windowed_witness_regression_v2__case_0__1171300000.mp4](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/windowed_witness_regression_v2__case_0__1171300000.mp4) | 성공 | 확인 | 236.793 |

</details>

<!-- SIMULATION_VIDEO_INDEX:END -->

## 마커 크기 무작위화

기존 배치는 마커 한 변12~30cm를 사용했습니다. 새 크기 변동 배치
`scenarios_robust/marker_sizes_v1`은 기본 **8~30cm** 범위에서 크기를 생성합니다.
크기는 검은 마커 정사각형의 한 변이며, 흰색 배경은 그1.25배입니다.
벽·장애물·가려진 면의 배치40개(학습용)와20개(검증용),총180개 마커를 생성했습니다.
각 마커의 부착면에 맞는 범위를3등분하고 작은·중간·큰 구간을 무작위로 배정합니다.
좁은 부착면에서는 상한이 작아지며, 각 마커에 실제 적용한 크기와 상한을 YAML에 기록합니다.

크기 난수 시드는 장애물 배치·ID·시작 위치와 독립적입니다. 한 회차 안에서는 물리적 크기가
고정되고 다음 배치에서 달라집니다. 실제 검출기는 정답 크기를 전달받지 않고 RGB·깊이·LiDAR를 사용합니다.
새 배치를 만들려면 기존 폴더와 겹치지 않는 출력 경로를 지정합니다:

```bash
PYTHONPATH=src/osp_mission python3 training_robust/prepare_marker_size_curriculum.py \
  --output scenarios_robust/marker_sizes_v2 \
  --minimum 0.08 --maximum 0.30 --size-mode mixed \
  --seed 9820000 --size-seed 19820000
```

`--size-mode uniform`은 각 마커의 허용 범위에서 독립 균등 샘플링합니다.
[크기 변동 Gazebo 검증 계획](repository_snapshot/marker_sizes.json)은5회이며 기존 수정본 회귀 검증 뒤에
실행합니다. 배치 생성과 테스트 통과는 실제 Gazebo 인식 성공을 뜻하지 않습니다.

## 이동·탐색 회전 비용 비교 후보(v14)

경로 로그에서 목적지 도착 후360도 관측 회전 시간이 점수 계산에 포함되지 않음을 확인했습니다.
최신 v13의 두 성공 사례는218.2초 중81.0초,210.4초 중54.9초를 거의 제자리에서 회전했습니다
(실제 궤적 기준 선속도0.02m/s 미만·각속도0.15rad/s 초과; 전부 불필요한 회전이라는 뜻은 아님).

`training_robust/native90/runtime_route_v14_staging`은 관측으로 얻을 정보의 점수에
예정된 회전 시간(`abs(scan_angle)/0.85`)을 비용으로 추가한 비교 후보입니다.
이동·방향전환 계수와 인식·접근·충돌 판정은 유지합니다.360도 관측도 정보 이득이 충분하면 선택합니다.
테스트192개가 통과했으며, 저장 입력20개를 다시 평가했을 때360도 선택은9개에서2개로 줄었습니다.
이 수치는 입력 재생 결과이며 실제 주행 개선이나 완주율 향상을 뜻하지 않습니다.
동일 배치3개에서 실제 Gazebo 비교를 진행합니다.
[상세 진단과 비교 계획](repository_snapshot/route_timecost.json)

## 추가 점검 결과 — 수정 전 확인 사항

[재현 기록](repository_snapshot/additional_issues.json)은 실행 중인 v14 소스를 변경하지 않고 만든 읽기 전용 진단입니다.

- **작은 마커 관측 완료 기준:** 크기 변동 배치는8cm까지 포함하지만 면 관측 신뢰도에는12cm 가정이 남아 있습니다.
  정면3.5m의8cm 마커는 약15.2px(검출 최소18px)인데도 합성 입력에서 면 관측 완료가 재현됐습니다.
  새로운 크기 범위의 실제 인식 성공을 아직 주장하지 않습니다.
- **지도 갱신 중 관측 기억 손실:** v14 실제 입력 재생에서 이전 관측 면62개 중6개만 유지된 구간이 있었습니다.
  SLAM 보정 후 일부 무효화는 필요하므로 모든 기억을 무조건 유지하는 수정은 적절하지 않습니다.
  중복 탐색에 미치는 실제 영향과 보존 조건은 추가 검증이 필요합니다.
- **상충 ID 보류의 해제 문제:** 관측 이력은 ID별30개로 제한되지만 우세 판정은3배를 요구합니다.
  같은 위치의 잘못된 ID 관측이11개 쌓인 조건에서는 정상 ID를100프레임 더 봐도30:11에 머물러
  둘 다 보류됩니다. 제어된 입력으로 재현했으며 완료된 실제 실험의 발생 사례로 확정한 것은 아닙니다.
- **상태 표시와 완료 판정 불일치:** 접근 상태 표시는 과거 마커 위치, 완료 판정은 현재 위치를 사용합니다.
  마커 추정 위치가10cm 수정되면 화면에는3개 접근 완료지만 내부 완료 조건은 거짓일 수 있습니다.
  이는 상태 표시 결함이며 잘못된 완주를 인정했다는 뜻은 아닙니다.

재현 명령:

```bash
python3 training_robust/audit_additional_issues.py \
  --workspace training_robust/native90/runtime_route_v14_staging \
  --campaign training_robust/native90/route_v14_timecost_screen3 \
  --output repository_snapshot/additional_issues.json
```

실제 정책 입력 스냅샷은 로컬 실험 산출물로 보관합니다. 새 체크아웃에는 요약 재현 기록만 포함됩니다.

## SLAM + Nav2 파라미터 학습 — 2026-11-09

`slam_nav2_search_v1`은 실제 Gazebo에서 `slam_toolbox`와 Nav2 RPP를 실행하는 개발용 탐색입니다.
v13 기준, 지도 갱신과 주행을 보수적으로 조정한 후보, frontier 가중치를 높인 후보의 세 조합을 비교합니다.
공통으로 센서 시각 비교를 ROS 시간으로 맞추고 관측 신뢰도의 최소 마커 크기를8cm로 수정했습니다.

장애물8/16/24개와 벽·장애물·뒷면 부착을 섞은 훈련 배치3개에서9회 비교하고,
1위 후보를 별도12/24/32개 배치3개에서 검증합니다. 마커 크기는8–30cm입니다.
최대2개 Gazebo 실행과 각6GiB 메모리 제한을 사용하며 영상과 평가 결과를 저장합니다.
평가는 정확한 ID3개, 각1.5m 접근, 좌표 오차0.5m 이하, 충돌0회,600초 이내 완료입니다.
후보 선택은 가중치와 SLAM/Nav2 설정의 제한된 파라미터 탐색이며 결과 확인 후 별도 배치 검증이 필요합니다.
독립40개 holdout은 사용하지 않습니다. 기본 실물 실행기는 계속 v13 후보를 가리킵니다.

[학습 시작 시점의 구성과 진행 상태](repository_snapshot/slam_nav2_search.json)

```bash
python3 training_robust/prepare_slam_nav2_search.py --output training_robust/native90/NEW_SEARCH
```

새 경로에서 후보별 `colcon build`를 마친 후 `native_tournament.py NEW_SEARCH`로 실행합니다.
이12회 비교는 종료됐습니다. 훈련 비교에서 기준0/3, 지도 안정화1/3, frontier2/3 완주였고,
frontier의 별도 검증은2/3 완주였습니다. 이후 학습은 아래의 새 맵 연속 학습으로 이어갑니다.
최근 완료된 v13 회귀 실험은3/6, v14 회전 비용 비교는2/3, 마커 크기 변동 실험은3/5 성공했습니다.
v14는 공통 성공 배치 두 개에서 v13보다 느렸습니다. 이 개발 배치 결과를 일반 완주율로 해석할 수는 없습니다.


## 불필요한 회전 감소 후보 — 2026-11-12

기존 관측은 시작 시360도, 이동 후120도 또는360도를 회전했습니다.
새 후보는 현재 카메라의 위치·방향·시야와 관측된 장애물 면을 이용해,
미확인 면을 보는 **짧은 방향의 최대120도 회전**을 고릅니다.
제한된 회전 중 얻을 수 있는 최대 관측량의90%를 유지하는 가장 짧은 각도를 선택합니다.
이미 카메라 정면에 있는 면과 가려진 면은 추가 회전 이득에서 제외하고,
새로 볼 면이 없으면 회전 없이1.5초간 영상을 처리합니다.
이동 실패·취소 후에도 도착한 것처럼 관측 회전을 시작하던 동작을 제거했습니다.
관측 완료 기록은 실제 카메라 처리에서만 갱신하며, 목표물 정답은 사용하지 않습니다.

검증은 기본 소스174개, v13에서 분리한 후보194개, 실물 어댑터4개 테스트 통과 및
후보 colcon 빌드 성공입니다. 합성 지도에서는 왼쪽/오른쪽 미확인 벽을 각각+105도/−105도로
관측하고, 이미 확인한 면은0도로 생략했습니다.
이는 **합성 입력과 코드 검증**이며 새 Gazebo 완주 결과나 실물 성능이 아닙니다.
기존 benchmark·영상·동결된 v13/v14 소스는 보존했습니다. 기본 실물 실행은 v13을 유지합니다.

새 후보를 별도 경로에 준비해 실행하려면:

```bash
bash run_robot_policy.sh --domain ROBOT_DOMAIN --low-rotation --build-only
bash run_robot_policy.sh --domain ROBOT_DOMAIN --low-rotation --rviz --start
```

`ROBOT_DOMAIN`은 실제 로봇 ROS 도메인 숫자로 바꿉니다. `--build-only`는 로봇을 움직이지 않습니다.
후보 소스는 `deployment/robot_low_rotation/`에 분리해 보관하며 재빌드 시 덮어쓰지 않습니다.
이 코드 변경 당시에는 세 마커의 정확한 ID·각1.5m 접근·올바른 지도 좌표·무충돌600초 완주를 검증하지 않았습니다.
이후 수행한 새 Gazebo 녹화 결과는 아래 항목에 별도로 기록합니다.


## 회전 감소 후보의 새 Gazebo 녹화 — 2026-11-12

장애물12개, 장애물 뒤에 숨긴 마커3개가 있는 개발 배치(seed8000001)에서
회전 감소 후보를 **새로 실행하고 녹화**했습니다. 재생·과거 로그의 재평가가 아닙니다.

- 완료 시간:494.534초(약8분15초), 접촉0회.
- 실제 ID283·337·388을 모두 인식했습니다.
- 평가용 정답 좌표에 대한 실제 최소 접근 거리:각1.216m·0.858m·0.562m로 모두1.5m 이내입니다.
- 지도 위치 오차:각7.1cm·1.8cm·1.9cm입니다(허용0.5m).
- 관측 회전 요청34회 중5회 생략, 나머지 요청은 모두 최대120도 이내였습니다.
  이는 관측용 회전 요청이며 Nav2 이동 중 방향 변경까지 합친 총 회전량은 아닙니다.
- 영상:1280×720,5fps,495초. 2,475프레임 전체 디코딩 및 세 영상 소스 수신을 확인했습니다.

한 개발 배치의 성공입니다. 같은 배치의 기존 후보와 대조 실험을 하지 않았으므로
시간 단축·완주율 향상을 입증한 것으로 해석하지 않습니다. 실물 기본 후보는 계속v13이며
production 승격은 하지 않았습니다. 평가용 정답은 정책에 전달하지 않았습니다.

[새 실행 결과와 영상 검증](repository_snapshot/low_rotation_video.json).
소유자의 이번 요청에 따라 공개 GitHub Release 업로드를 허용받았습니다.

[이번 회전 감소 후보의 8분15초 영상 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/low_rotation_video_20261112__episode_8000001.mp4)

## 매 훈련 회차 새 맵을 사용하는 연속 학습 — 2026-11-12

`fresh_map_cem_v1`은 매 훈련 실행마다 새 seed로 실제 장애물 지도와 마커 배치를 생성합니다.
PDF의20.15×11.35m 외곽은 유지하고 장애물8/12/16/24/32개, 위치·방향, 부착면,
마커 ID·8–30cm 크기, 도달 가능한 출발 위치·방향을 바꿉니다.
장애물 뒤 배치는 마커 정면이 시작 위치에서 보이지 않는 쪽으로 출발 위치를 제한합니다.
환경 seed와 물리 배치 hash를 기록하고 훈련 맵 재사용을 거부합니다.
준비한 첫20개 회차의 seed와 물리 배치 hash는 모두 서로 다릅니다.
별도 검사에서도 ID·seed를 제외한0.10m 해상도 장애물/지지대 점유 격자가20개 모두 달랐고,
출발 위치도20개 모두 달랐습니다. 이는 준비된 환경의 기하 검사이며 실행 완료 횟수가 아닙니다.
[점유 격자 기반 다양성 검사](repository_snapshot/fresh_map_diversity.json).

한 세대는 탐색 가중치 후보4개×각5개의 서로 다른 맵으로 구성됩니다.
Gazebo에서 SLAM과 Nav2를 실행한 평가 결과로 상위 후보2개의 로그 가중치 분포를 갱신하고,
다음 세대에서 새 후보·새 맵을 생성합니다. 초기 가중치는 이전 frontier 후보에서 가져왔고,
회전 감소 코드와8cm 관측 기준을 사용합니다.
실패는1,200초, 완주는 실제 소요 시간을 반영하는 PAR-2 시간 비용으로 평가합니다.
파일럿 검증은 별도5개 맵에서 후보와 이전 frontier 후보를 같은 조건으로 비교합니다.
이 비교에서4/5 이상 완주·접촉0·시간 비용20% 감소를 충족하면20개 배치의 짝지은 검증으로 넘어갑니다.
20개 검증에서18/20 이상 완주·접촉0·시간 비용20% 감소 후 독립40개 최종 평가를 수행합니다.
검증 맵만 후보와 기준 정책에 동일하게 제공하며 정책은 정답 좌표를 받지 않습니다.
최종 평가에서 실패한 배치도 훈련에 재사용하지 않고 이후 새 독립 배치로 평가합니다.

`marker_events.jsonl`에 ID 확정·철회,1.5m 접근 추정·철회, 임무 상태 변경을 누적합니다.
별도 정답 기반 접근 거리·지도 좌표·ID 감사는 `episode_audit.json`, 영상은 `video/mission.mp4`,
각 회차의 가중치·결과는 `learning_log.json`과 `state.json`에 보관합니다.
최대2개 native 실행과 각6GiB 제한을 유지합니다. 인프라 실패3회 연속 또는 디스크8GiB 미만이면
원인을 기록하고 수리를 기다립니다. 정상 성능 비교는 다음 세대와 새 맵으로 계속 진행합니다.

이번 준비 과정의 검증은 runtime197개 통과·1개 skip, 학습·다양성·시간 평가·마커 로그6개 통과와
colcon 빌드 성공입니다. 새로운 맵에서의 성능 향상은 아직 검증 중이며 기본 실물 정책은 승격하지 않았습니다.
[현재 구성과 검증 범위](repository_snapshot/fresh_map_curriculum.json).
실행 서비스는 `osp-fresh-map-cem-v1.service`이며 이전12회 SLAM/Nav2 실험은 종료됐습니다.
이전 우승 후보의 별도 검증 결과는2/3 완주(287.908초·428.354초, 나머지600초 timeout)였습니다.

```bash
# 이전 SLAM/Nav2 비교의 frontier runtime이 빌드된 환경에서 새 실행 경로를 준비합니다.
python3 training_robust/prepare_continual_native.py training_robust/native90/NEW_FRESH_MAP_RUN
# 생성된 runtime에서 colcon build를 마친 뒤 감독기를 실행합니다.
python3 training_robust/continual_native_curriculum.py training_robust/native90/NEW_FRESH_MAP_RUN
```

## 작은 마커 후보 재확인 실험 — 2026-11-12

새 훈련 맵의 첫2회에서10–17px 후보가 반복 검출돼도18px 인식 기준 때문에
근접 관측 후보로 전달되지 않는 동작을 확인했습니다. 별도 후보 runtime에서는10px 이상 후보를
거리·수직면·3프레임 위치 일관성으로 걸러 가까이서 확인할 관측 힌트를 발행합니다.
인식은 기존과 같이18px 이상 관측 이력3개로 확정합니다.
작은 후보는 인식 이력·마커 지도·접근 완료 판정에 들어가지 않습니다.

현재2회 Gazebo 주행은 끝까지 유지하고, 이후 같은 맵·같은 가중치에서 진단 비교2회를 실행합니다.
이 비교는 훈련 샘플에 넣지 않습니다. 비교 후 연속 학습 감독 서비스를 자동으로 재개합니다.
systemd 임시 서비스 정의가 정지 후 사라지면 같은 설정으로 감독 서비스를 다시 생성합니다.
동시 Gazebo 실행은 최대2개, 각6GiB입니다.

runtime 테스트200개 통과·1개 skip, 감독 서비스 테스트2개 통과와 colcon build 성공입니다.
14px 영상은 관측 힌트만 생성하고,24px 영상도3회 관측 후 ID를 확정하는 경로를 검증했습니다.
실제 Gazebo에서의 시간 단축·완주 개선은 추가 검증이 필요합니다.
[구성과 진단 비교 범위](repository_snapshot/small_marker_followup.json).

## 첫 새 맵 회차 결과와 Nav2 시간 오류 — 2026-11-12

첫2회 새 맵 훈련은 모두600초 시간 초과였습니다. 첫 회차는 ID3개와 지도 좌표가 모두 정확했고
좌표 오차는1.5–3.7cm였지만, 실제 최소 접근 거리가1.28m·1.43m·4.53m여서2개만 접근했습니다.
두 번째는1개 인식·접근이었고 두 회차 모두 접촉0회였습니다. 해당 영상도 Release에 저장했습니다.

첫 회차에는 Nav2 목표 실패63회와 로봇 위치→전역 경로 좌표 변환 오류72회가 있었습니다.
미래 시각 외삽 오류의 최대 차이는0.184초였습니다. 두 번째는 목표 실패20회·좌표 변환 오류31회였습니다.
이는 탐색 가중치 외에 주행을 반복 중단시키는 시간 문제가 있음을 보여 줍니다.

설치된 SLAM Toolbox2.8.5와 같은 버전의 공식 구현에서 `restamp_tf`가 현재 시각과
`transform_timeout`으로 map→odom TF 시각을 발행하는 것을 확인했습니다.
[SLAM Toolbox2.8.5 구현](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.8.5/src/slam_toolbox_common.cpp).
이 설정만 켠 별도 후보를 빌드했고, TF buffer의 실제4ms 외삽 실패와 시각 범위 보완 후 성공을 재현했습니다.
후보 runtime 테스트198개 통과·1개 skip입니다. 실제 주행 개선은 아직 검증하지 않았습니다.

작은 마커 진단 비교가 끝나고 새 맵 학습의 다음2회 주행이 활성화된 것을 확인한 뒤,
그 주행을 끝까지 유지하고 TF 설정 후보의 같은 조건 진단 비교2회를 수행하도록 대기시켰습니다.
이후 연속 새 맵 학습 감독기를 다시 재개합니다. 회차·맵·마커·가중치·영상 기록을 보존하며
동시 Gazebo 실행은 최대2개입니다.
[실패 로그 집계와 TF 후보](repository_snapshot/tf_navigation_failure.json).

작은 마커 재확인 후보의 두 진단 비교는 모두 독립 감사를 통과했습니다.
기존600초 timeout 두 배치를 같은 맵·같은 가중치에서 각각282.519초와193.2초에 완주했습니다.
세 ID·각1.5m 실제 접근·지도 오차0.5m 이하·접촉0회를 모두 충족했고 영상도 게시했습니다.
이 결과는 이미 관찰한 두 배치의 진단 개선이며, 새 맵 전반의 완주율 검증은 계속 필요합니다.
비워진 자리에 새 맵 훈련을 재개했고, TF 비교 감독기가 활성화된 새 훈련 주행을 끝까지 유지한 뒤
시간 설정 후보를 비교합니다. 현재 상태는 각 감독기와 로컬 state 파일로 확인합니다.

## 검증된 재확인 코드를 다음 새 맵 학습으로 이전 — 2026-11-12

두 진단 비교에서 성공한 코드를 `fresh_map_cem_v2`의 출발점으로 삼는 전환 서비스를 준비했습니다.
TF 비교 종료 후 실제 오류 수·시간 비용·접촉·좌표 결과로 `restamp_tf` 반영 여부를 정합니다.
TF 설정은 짝지은 PAR-2가 개선되고, 접촉·인프라 실패가 없으며, 각 배치의 로봇 위치 변환 오류가
절반 이하이고 관측된 ID·좌표가 평가 기준을 충족할 때 함께 적용합니다.
이후 결합 runtime을 빌드·테스트하고 현재 주행을 끝까지 유지한 뒤 감독기를 교체합니다.

이전 실행 결과는 보존하고 최신 가중치 분포를 이어받습니다. 완료된 훈련 세대가 있으면 그 결과도
분포 갱신에 반영합니다. 새 학습은 seed1123200000부터 시작하고, 첫20개 맵의 점유 격자가
새 세대 내부와 이전 학습 맵 사이에서 겹치지 않는지 검사합니다.
최대2개 native 실행·각6GiB, 시간 비용과 정확한 ID3개·실제1.5m 접근·좌표0.5m·무접촉·600초 기준을 유지합니다.
최종 독립 평가에 이미 사용한 배치를 다시 쓰지 않도록 사용 여부도 이전합니다.

준비·TF 선택·안전한 감독기 전환·시작 실패 복구 테스트5개가 통과했습니다.
재확인과 TF 시간 변경을 결합한 소스는 colcon 빌드와201개 테스트 통과·1개 skip을 확인했습니다.
새 맵 개선 효과는 새로운 CEM 학습과 짝지은 검증으로 평가합니다.
실제 전환은 `osp-fresh-map-v2-upgrade.service`, 진행 기록은 기존 campaign의
`successor_transition.json`과 `successor_transition.log`에서 확인합니다.
기존 코드의 추가 새 맵 회차(seed1121200303)는317.4초 완주와 독립 감사를 통과했습니다.
317.4초 회차는 기존 탐색 코드를 사용했습니다.
[학습 전환 계획·검증·현재 관측 결과](repository_snapshot/fresh_map_successor.json).

## 외곽 공간 다양화와 재방문 억제 — 2026-11-13

`spatial_cem_v1`은 각 세대에서 후보 4개를 서로 다른 공간 7종씩, 총28개 실제 Gazebo
주행으로 평가합니다. 공간 크기·외곽 다각형·방 연결·출발점·장애물·마커 부착 면과
ID·8–30cm 크기를 함께 바꿉니다. 원형의 충돌 벽은 닫힌48분할 다각형 근사입니다.
첫28개는 **외곽 형상28개, 점유 지도28개가 모두 고유**하고 세 마커의 접근 가능성을
오프라인 기하 검사로 확인했습니다. 이 검사는 완주 성능을 증명하지 않습니다.

새 후보는 실제 ROS odom에 주행 이력을 고정하여 SLAM 지도 좌표 보정 이후에도 방문 이력을
유지합니다. 최근 성공적으로 도착한 같은 위치·같은 관측 방향의 저정보 탐색을 억제하고,
새 frontier·다른 관측 방향·마커 확인/접근은 허용합니다. 통로를 다시 지나는 행위는 금지하지
않습니다. `exploration_memory.jsonl`에 이동 거리·재방문 추정 거리·억제한 후보 수를 남깁니다.
재방문 거리에는 필요한 통과도 포함되며 그대로 낭비 거리로 해석하지 않습니다.

형태별 약점이 전체 평균에 숨지 않게 파일럿7종, 쌍대 검증28개(각 형태 최소3/4),
새 공간 독립 시험42개(각 형태 최소5/6)와 원래 PDF 공간 독립 시험40개를 순서대로 둡니다.
쌍대 검증은 성공률90% 이상, 접촉0회와 이전 정책 대비 PAR-2 시간20% 이상 개선을 요구합니다.
각 임무는 정확한ID3개·실제1.5m 접근·지도 오차0.5m 이하·접촉0회·600초 이내를 따로 감사합니다.
Gazebo 동시 실행은 최대2개이며 각 실행의6GiB 메모리 제한을 유지합니다.

기존 실행과 소스는 보존합니다. 활성 캠페인의 `configuration.json`에 등록한 runtime과
driver/환경 생성기 해시는 고정됩니다. 추가 변경은 새 후보/새 버전으로 준비해야 합니다.
새 후보와 비교 기준 runtime은 각각 **210개 테스트 통과·1개 skip 및 빌드 성공**입니다.
[형태별 구성과 실행 증거](repository_snapshot/spatial_curriculum.json) ·
[재방문 분석의 범위](repository_snapshot/revisit_analysis.json).

## 다각형 바닥 좌표 오류 수정과 입력 검증 — 2026-11-13

초판 공간의 OBJ 바닥 꼭짓점은 월드 좌표였지만, floor 모델의 중앙 좌표와 높이(-0.06m)가
다시 적용됐습니다. 충돌 바닥과 GPU 센서에 보이는 바닥이 어긋나 실제 원형·삼각형 모두
바닥 센서4개가 전부 무한대를 반환했고 낭떠러지 이벤트가 발생했습니다. Create3 자체 보호
동작이 Nav2 속도 명령을 무시하며, 삼각형은 벽 근처에서 탐색 후보가 없어지는 복구를
반복했습니다. 이는 정책 성능으로 사용할 수 없는 시뮬레이션 입력 오류입니다.

두 실행은 `/mission/stop`으로 종료하여 원본 평가·지도·영상·로그를 보존했습니다.
`operator_stop` 결과를 성공/시간 초과로 바꿔 쓰지 않고 `environment_invalidation.json`으로
학습 표본 제외를 표시합니다. 대시보드도 ‘환경 오류로 종료’로 구분합니다.
낭떠러지 감지나 충돌 보호 기능을 끄지 않았습니다.

새 `floor_pose_v1` runtime은 바닥 visual에 모델 이동을 상쇄하는 상대 pose를 넣습니다.
7종 공간의 모든 바닥 꼭짓점과 충돌 바닥 상면이 실제 월드 z=0에 일치하는 검사를 추가했고,
runtime 전체 **217개 테스트 통과·1개 skip 및 빌드 성공**을 확인했습니다.
동일한 실제 원형·삼각형을 재실행한7초 관측에서4개 센서가100% 유효 거리를 반환했고
낭떠러지 이벤트는0회였습니다. 두 완주 회귀 실험은 별도로 기록합니다.

`spatial_cem_v3`는 새로운 외곽28개를 준비하고 기존 실행이 비우는 슬롯에서 계속 학습합니다.
각 실행에 실제 바닥 센서4개와 hazard 스트림 검사를 요구하며, 입력이 검증되지 않으면
완주로 보이는 결과도 학습 성공으로 인정하지 않습니다. 현재 동시 Gazebo 상한은2개입니다.
처음 수정한 `nav2_robot_policy.yaml`은 Gazebo 기본 실행 경로가 아니어서 실제 응답 대기는
여전히20ms였습니다. 이 후보와 현재 주행은 보존하고, 새 후보에서 실제 경로인
`nav2_gazebo.yaml`도 수정했습니다. 새 실행의 ROS GetParameters로 **1000ms 실제 적용**,
현재 정책 파일·600초 제한·SLAM restamp_tf=true를 확인했습니다.
각 주행마다 실제 적용값을 다시 확인하며, 기준 정책의20ms와 후보1000ms를 구분합니다.
응답 대기의 시간 개선 효과는 실제 쌍대 검증 대상이며 아직 입증하지 않았습니다.
원형·삼각형 진단 결과만으로7종 공간 일반화나 실사용 정책 완성을 주장하지 않습니다.

수정한 바닥의 두 동일 배치 회귀 주행은 각각 **원형288.903초, 삼각형427.501초**에
세 ID·실제1.5m 접근·좌표 오차0.5m 이내·접촉0회를 독립 감사로 통과했습니다.
원형의 최대 지도 오차는0.460m로 허용치에 가까워 다른 배치에서도 충분한지 확인해야 합니다.
이는 두 진단 배치의 결과이며 전체 성공률·학습 완료·응답 대기 변경 효과를 의미하지 않습니다.

TF 설정만 바꾼 진단 비교도231.867초·229.859초에 모두 완주했고, 독립 감사를 통과했습니다.
관련 로봇 위치 변환 오류는 기존72회·31회에서 두 비교 모두0회로 줄었습니다.
실제 SLAM 노드에서 `restamp_tf=true`가 적용된 것도 ROS 파라미터 조회로 확인했습니다.
시간 비용·좌표·ID·접촉 기준을 충족해 다음 학습에는 재확인 코드와 이 시간 설정을 함께 넣기로 선택했습니다.
선택된 실제 다음 runtime도 빌드와201개 테스트 통과·1개 skip을 마쳤고,
현재 활성 주행을 종료한 뒤 새로운20개 맵의 학습을 시작하도록 전환 서비스가 진행 중입니다.
이 두 비교는 기존 배치의 진단 결과이며, 결합 후보의 새 맵 일반화 성능은 계속 검증합니다.
