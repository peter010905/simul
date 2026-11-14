# TurtleBot4 Standard Jazzy platform candidate

실물과 같은 ROS 인터페이스로 SLAM·Nav2·인식을 실행하는 후보입니다. 실제 기체의 내부 펌웨어·센서 보정값을 복제한 디지털 트윈이나 실사용 검증 정책은 아닙니다. 원래 과제의 Standard 모델을 사용하며, 사용자가 확인한 기종 범위는 TurtleBot4입니다.

| 항목 | 이 후보의 구성 |
| --- | --- |
| 위치추정 | 실물: Create 3의 `odom`과 `odom → base_link` TF 사용. Gazebo: 공식 wheel encoder odometry를 20 Hz로 발행. 별도 EKF·시뮬레이션에서 맞춘 휠 보정 계수 제거 |
| 지도 | 공통 `tb4_slam.yaml`, SLAM Toolbox async/sync·pose graph 저장·재사용·localization 지원 |
| 주행 | 공식 Jazzy MPPI 기반 `tb4_nav2.yaml`을 실물/시뮬레이션에 공통 적용. 기본 속도 제한·velocity smoother·collision monitor 유지 |
| 명령 | 최종 `cmd_vel`: `geometry_msgs/msg/TwistStamped`. republisher YAML만 보면 `cmd_vel_stamped`지만 공식 **robot.launch.py가 다시 cmd_vel로 remap**함 |
| 카메라 | 기본 OAK-D RGB preview 250×250 / 30 Hz. 실제 `CameraInfo`로 계산. Gazebo 전용 depth 입력을 정책에서 제외하고 RGB + LiDAR 평면 거리 사용 |
| LiDAR·IMU | A1M8 명목 5.5 Hz, 0.15–12 m, 합성 1440 samples; IMU 100 Hz. 실물 개별 스캔 수·잡음·FOV를 측정한 값은 아님 |
| 시작 검사 | odom/TF 단일 발행 주체, stamped 명령 소비자, 센서 시각·주기·카메라 보정·TF 연결·hazard 수신 검사. 누락/충돌 시 대기, 주행 중 끊기면 취소·정지 후 재개 |
| 접근 판정 | 원래 세 ID·실제 1.5 m 접근·지도 오차 ≤0.5 m·충돌 0·600초 기준 유지. 거리 근거에 `range_source`와 `independent_depth_lidar`를 기록 |

실제 Create 3는 wheel encoder·IMU·optical mouse를 융합합니다. 설치된 Harmonic 모델의 encoder odometry는 이 비공개 융합을 재현하지 않습니다. 바닥 마찰·미끄럼·노출·가림·네트워크와 Raspberry Pi 처리 지연도 동일하다고 주장하지 않습니다. 카메라 FOV는 upstream 모델에서 정사각형 crop을 가정해 계산한 명목값이며 기체 보정값이 아닙니다.

TF 검사는 odom 발행자와 TF 그래프 endpoint의 소유자를 대조하고 odom edge 수신·예상하지 않은 TF 발행자를 검사합니다. Jazzy rclpy가 메시지별 GID를 제공하지 않아 메시지별 발행자 감사는 아닙니다.

`upstream/`에 참고 설정 원문, `provenance.json`에 버전·출처, `candidate.json`에 파일 해시를 보관합니다. 활성 SLAM 설정은 공식값에 `restamp_tf=true`를 더했고, Nav2는 액션 응답 대기를 1000 ms로 늘렸습니다. 이 값들은 실물·시뮬레이션 공통입니다. 과거 소스의 EKF/helper/config 파일은 원본 재현 때문에 빌드 안에 남지만 새 launch는 이를 실행하지 않습니다.

## 재현

원래 release + robot_transfer_v1 + 이 overlay 순서로 새 작업 공간을 만듭니다. 기존 작업 공간을 덮어쓰지 않습니다.

```bash
cd /home/peter/simul
python3 policy_candidates/2026-11-14_tb4_platform_v1/build.py \
  --workspace /home/peter/simul/deployment/tb4_platform --test
source /opt/ros/jazzy/setup.bash
source /home/peter/simul/deployment/tb4_platform/install/setup.bash
ros2 launch osp_mission simulation.launch.py camera_width:=250 rviz:=true
```

실물에서 이미 기본 bringup이 실행 중이면:

```bash
ros2 launch osp_mission hardware.launch.py bringup:=false rviz:=true
ros2 topic echo /mission/platform_status --once
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

공식 Jazzy republisher 기본 파일은 `hazard_detection`을 주석 처리합니다. 이 임무는 충돌·위험 입력이 필요하므로 `config/tb4_republisher.yaml`에서 해당 발행을 활성화했습니다. 기존 bringup에 이 파일을 `create3_param_file`로 전달하거나, 기존 bringup을 종료한 뒤 `hardware.launch.py bringup:=true`를 사용합니다. 후자는 공식 Standard bringup에 이 파일을 전달합니다. 이미 실행 중인 bringup과 중복 실행하면 안 됩니다. 여기서 실제 로봇의 서비스나 설정을 원격 변경하지 않았습니다.

로봇의 `ROS_DOMAIN_ID`를 사용하고, namespace가 있다면 `namespace:=robot1`을 지정합니다. 이때 상태/시작 서비스도 `/robot1/mission/...`입니다. 다른 SLAM/AMCL/Nav2를 동시에 실행하지 않습니다. `platform_status.errors`가 대기 이유를 보여주며, 형식이 다른 토픽에 추측으로 명령을 보내지 않습니다.

- 동기 SLAM: `slam_mode:=sync` (기본 async).
- 지도 업데이트 재개: `session:=continue posegraph:=/absolute/path/map resume_pose:=x,y,yaw`.
- 저장 지도 위치추정: `session:=localization posegraph:=/absolute/path/map resume_pose:=x,y,yaw`.
- 종료 시 `map.pgm`, `map.yaml`, `map.posegraph`, `map.data`, `markers.csv` 저장. PGM/YAML만으로 pose graph를 대체하지 않습니다.

## 검증 범위

373개 런타임 검사·3개 감독기 설정 검사·1개 실제 ROS 전송 통합 검사 통과. 새 native 검증 상태는 [상태 기록](../../repository_snapshot/tb4_platform_candidate.json)을 확인합니다. 짧은 platform 주행 검사는 전체 임무 성공률이 아닙니다. 새 학습은 동일한 플랫폼에서 계수만 비교하고, 과거 960×720 + depth + EKF 실험과 직접적인 정책 개선 비교를 하지 않습니다. 실제 로봇 검증은 미완료입니다.
