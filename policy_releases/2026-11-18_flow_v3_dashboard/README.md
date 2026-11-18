# 실물 대시보드 Flow v3 · MPPI

2026년 11월 18일 실물 보드의 **Flow v3 · MPPI 구동** 버튼에서 실행한 작업 공간 `face_intent_v3_sweep_ws`의 소스와 설정이다. 마지막 주행 `20261118T084307-e58115524e`에서 103번 마커의 확정·접근을 기록했다. [보드 주행 결과](https://github.com/peter010905/simul/releases/download/policy-logs-robot-2026-11/2026-11-18-flow-v3-surface-sweep-board-results.md)

| 항목 | 위치 |
|---|---|
| 탐색·주행·복구 | [mission.py](src/osp_mission/osp_mission/mission.py), [MPPI 설정과 진행 판단](src/osp_mission/osp_mission/transit_control.py) |
| 표면을 훑는 회전 | [surface_sweep.py](src/osp_mission/osp_mission/surface_sweep.py), [표면 선택과 반복 경로 감점](src/osp_mission/osp_mission/wall_route.py) |
| Original 마커 인식 | [detector.py](src/osp_mission/osp_mission/detector.py), [인접 ID 판별](src/osp_mission/osp_mission/marker_pair.py), [왜곡 보정](src/osp_mission/osp_mission/warped_marker.py) |
| face_intent_v2 가중치 | [policy.json](policy_releases/2026-11-13_face_intent_v2/policy.json) |
| 실제 시작·정지 및 준비 검사 | [robot_session.py](scripts/robot_session.py), [ROS 런치](src/osp_mission/launch/robot.launch.py) |
| 실물 설정 원본 | [유선 DDS·Nav2·시간 동기화](robot_profiles/wired_flow_v3/) |
| 해당 런치에서 생성한 설정 | [Nav2](applied_config/nav2.yaml), [SLAM](applied_config/slam.yaml) |

## 복원과 확인

실행 환경은 ROS 2 Jazzy, TurtleBot4 Navigation 2.1.1, Nav2 MPPI, slam_toolbox, OpenCV ArUco 및 Python ROS 패키지가 설치된 Ubuntu VM이다. 이 디렉터리에서 다음 명령으로 네이티브 모듈과 ROS 패키지를 빌드하고 회귀 검사를 실행한다.

```bash
sha256sum -c SHA256SUMS
bash build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/robot_session.py dry-run --repo "$PWD" --workspace "$PWD" --domain 4 --duration 600
```

실물 보드는 ROS 도메인 4, Fast DDS 유선 프로필, 탐색기 `edge`, 600초 제한, 출발 반경 0.30m, Original 사전과 18픽셀·3프레임 기준으로 실행했다. 실제 시작 명령은 위 `dry-run`의 동작을 `start`로 바꾸고 아래 환경을 적용한다. 시간 서버와 DDS 인터페이스 설정은 해당 실험망 기준이다.

```bash
export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/robot_profiles/wired_flow_v3/fastdds_vm.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

원본 가중치 SHA-256은 `ca67b32c20a17e74e0bc8292b117240e19d80f24137cbf092d473a743be5eb36`이다. 런치가 참조한 TurtleBot4 Navigation 2.1.1의 `config/nav2.yaml`은 `f936bdee74f02e497317f31621b8d0013ff0f592b5d8ba7d570952bd33f5821f`이다.
