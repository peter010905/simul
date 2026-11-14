# Robot transfer candidate — 2026-11-14

실물에서 보고된 열린 길 오판·반복 탐색에 대응하는 **검증 중 후보**입니다. 실물 로그/센서 사양을 아직 받지 못했고, 실물 성공이나 최적 정책을 주장하지 않습니다. 기존 `face_intent_v2` 릴리스는 보존합니다.

- 시작점이 작은 inflation 섬에 갇혔을 때 최대0.8m의 관측된 빈 공간 연결을 찾습니다. 매 셀에서 반경0.189m+셀 반대각선+1cm 여유를 요구합니다. 장애물·미관측 셀은 열지 않으며 목적지의0.32m 여유 조건도 유지합니다. 이 몸체 치수는 TurtleBot4 Standard 가정입니다.
- odom 좌표의 성공/실패 목적지를 기억합니다. 같은 위치·방향의 변하지 않은 frontier 점수는 재방문 면제 사유가 아닙니다. 실패 재시도는15–90초, 성공한 중복 관측은150초 대기합니다. 새 frontier 이득·다른 관측 방향·마커 확인/접근과 필요한 통로 재통과는 허용합니다.
- 모든 후보가 걸러졌을 때 전체 후보를 되살리지 않습니다. 짧게 기다리며 지도 갱신을 재평가합니다. `planning_diagnostics.jsonl`에서 시작점 미관측/장애물/연결 실패와 관측 재시도 대기를 구분합니다.
- 실물 `robot.launch.py` 기본 탐색기와 정책을 `edge`로 맞추고 `nav2_robot_policy.yaml`을 선택합니다. odom 방문 이력은 설정된 `odom_frame`을 사용합니다. 실제 footprint·TF·센서 topic 설정은 장치에서 확인해야 합니다.
- 시뮬레이션에만 scan relay와 encoder 오차를 적용합니다. `none`, `mild`, `moderate`는 합성 스트레스이며 특정 로봇의 측정값이 아닙니다. LiDAR 표준편차0/1/2cm, 편향0/+0.5/−1cm, ray 누락0/1/3%, frame 누락0/2/5%, 지연0/60/120ms, 전진 odom 배율1/.98/1.03, 회전 배율1/1.01/.98입니다. 원래 취득 시각과 invalid 값은 유지합니다.
- SLAM과 시뮬레이션 EKF의 통신을 UDP로 한정하는 실험 설정이 포함됩니다. 초기2회 map/odom TF 준비를 통과했지만 과거 startup 실패의 원인이 이것이었다고 입증한 것은 아닙니다.

런타임357개 테스트, 동일 입력을 쓰는 이전 탐색 기준348개 테스트가 통과했습니다. `candidate.json`은 기본 릴리스와 변경 파일 해시를 기록합니다. 별도 디렉터리에 재현합니다:

```bash
python3 policy_candidates/2026-11-14_robot_transfer_v1/build.py \
  --workspace "$PWD/deployment/robot_transfer_candidate" --test
source deployment/robot_transfer_candidate/install/setup.bash
```

실물 실행은 그 로봇의 Jazzy 드라이버·센서·TF가 이미 가동된 환경에서 아래 공유 launch를 사용합니다. 이 launch는 `/mission/start` 전까지 임무를 시작하지 않습니다. 후보는 실물 배포 승인/검증 완료본이 아닙니다.

```bash
ros2 launch osp_mission robot.launch.py use_sim_time:=false planner:=edge \
  policy_file:="$PWD/policy_releases/2026-11-13_face_intent_v2/policy.json" \
  output_dir:="$HOME/simul/results/robot_transfer"
```

실제 문제 자료를 모으는 읽기 전용 도구(이동 명령을 발행하지 않음):

```bash
python3 scripts/capture_robot_diagnostics.py \
  --output "$HOME/simul/results/real_failure_01" --duration 90 --domain 0
```

실제 ROS 도메인·namespace·scan topic이 다르면 해당 인자를 지정합니다. rosbag은 로컬에 보관하고 Git에 추가하지 않습니다. 현재2개 native Gazebo 실행 외에 수동 시뮬레이션을 추가로 띄우지 않습니다.

학습은 `robot_transfer_cem_v2`에 기록합니다. v1의 관측 도구가 잘못된 ROS context executor를 사용한 문제를 별도v2로 수정했고, 기존 두 Gazebo 프로세스·runtime·준비된12개 배치를 그대로 인계했습니다. 모든 실행은 바닥·실효 설정·encoder·SLAM·합성 오차 적용 검사와 독립 물리 접근 감사를 거칩니다. 비교 기준에도 동일 센서 스트레스와 통신 설정을 사용합니다. 학습 계수 후보4개×3형태 이후 같은 맵/잡음 seed의 쌍대 검증을 수행합니다. 정답 마커 좌표·ID는 정책에 주지 않습니다.
