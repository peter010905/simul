# TurtleBot4 실물·시뮬 주행 보드

기존 `dashboard/server.py`는 Gazebo 캠페인용입니다. 이 보드는 실물 로봇의
ROS 토픽을 직접 구독합니다. 기본 실행은 읽기 전용이며, 실물 전용
`--enable-control`을 지정하면 상단 **구동·정지 버튼**을 사용할 수 있습니다.
미션 상태, 출발 지도 셀, 충돌 감시, 원본/필터 LiDAR, RGB, 지도와 Nav2 경로를
1초마다 표시합니다. 데이터가 끊기면 마지막 수신 시각과 경고를 보여줍니다.
지도 이미지는 카드 안에 전체가 들어오도록 맞추며, `전체 화면` 버튼으로 확대할 수 있습니다.
`/targets`에서 탐지 확인된 마커 ID를 받으면 `발견한 마커` 카드에 계속 표시하고,
새 ID는 15초 동안 알림으로 강조합니다. `접근 완료`는 별도 미션 상태의 1.5m
접근 확인을 뜻하므로 발견 수와 구분됩니다.

로봇과 같은 ROS 네트워크에 있는 Ubuntu/Jazzy 노트북에서:

```bash
cd /path/to/simul
source /opt/ros/jazzy/setup.bash
source /path/to/osp-v21/install/setup.bash
python3 dashboard/robot_board.py --domain 4 --host 0.0.0.0 --port 8766
```

Mac 브라우저에서 `http://<노트북-LAN-IP>:8766`을 엽니다. ROS 도메인이 4가
아니면 `--domain`을 바꾸고, 로봇 네임스페이스가 있으면 `--namespace tb4`를
추가합니다. 원본 스캔과 RGB 토픽이 다르면 `--scan`과 `--rgb`를 지정합니다.
기본 바인딩은 `127.0.0.1`이며, 외부 접근이 필요할 때만 같은 신뢰할 수 있는
LAN에서 `--host 0.0.0.0`을 사용합니다. 인터넷에는 공개하지 마세요.

`osp_mission` 설치 경로가 화면 아래에 표시됩니다. v21 필터를 확인하려면
`scan_self_filtered`의 마지막 수신 시각과 **몸체 내부 제거** 수를 확인하세요.
원본 스캔의 19cm 반사가 로봇 중심 기준 16cm 영역 밖이면 필터는 제거하지
않습니다. 출발 지도 셀이 계속 `start_not_free`라면 보드의 근접 LiDAR 화면과
지도/TF를 함께 살펴야 합니다. 저장된 점유 흔적을 제외하려면 새 SLAM 세션을
사용하세요.

보드는 ROS 2 Jazzy, `rclpy`, `nav2_msgs`, `cv_bridge`, OpenCV, NumPy가 필요합니다.
`/map`, `/scan`, `/scan_self_filtered`, `/mission/status`,
`/collision_monitor_state`, RGB 이미지, `/plan`, `/local_plan`, TF를 구독합니다.
실물에서 확인한 값과 시뮬레이션 평가 점수는 별개입니다.

## 이 Mac의 실물 제어

구동 버튼은 같은 정지 버튼과 시간·센서 점검을 사용합니다.

| 버튼 | 주행 정책 | Mac 명령 |
|---|---|---|
| Flow v3 · MPPI 구동 | 11월 17일 Flow v3의 탐색·마커 판정과 MPPI 경로 추종 | `./robot.sh start-v3-mppi` |
| Flow v4 구동 | 기존 Flow v4r2 | `./robot.sh start` |
| R32 혼합 구동 | R32 탐색과 Original/18 마커 판정 | `./robot.sh start-r32` |

준비된 Flow v4 런타임과 유선 DDS·NTP 설정을 재사용합니다. 브라우저에서
<http://192.168.252.2:8767/>을 열고 **구동**을 누르면 시간·센서·실제 범퍼·도크
상태 확인, SLAM/Nav2 실행, 준비 직후 미션 시작까지 자동 처리합니다.
재빌드나 수동 시작 서비스 호출은 없습니다. **정지**는 준비 중인 요청도 취소하고,
주행 프로세스 종료·액션 취소·0 속도 명령 후 실제 odom으로 정지를 확인합니다.
속도를 수신하지 못하면 정지 확인 실패로 표시합니다. 도크에 있으면 자동 출발하지
않고 도크 밖으로 옮기라는 이유를 표시합니다.

현재 VM에서 제어 보드를 실행하는 명령은 아래와 같습니다. 실행 환경에 유선
`FASTRTPS_DEFAULT_PROFILES_FILE`과 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`가 필요합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/face_intent_flow_v4r2_ws/install/setup.bash
export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/simul/robot_profiles/wired_flow_v3/fastdds_vm.xml
python3 -u dashboard/robot_board.py --domain 4 --host 0.0.0.0 --port 8767 \
  --enable-control --control-workspace /home/ubuntu/face_intent_flow_v4r2_ws
```

버튼은 같은 보드 페이지에서 보낸 토큰 포함 POST만 받습니다. GET·페이지 새로고침은
주행을 시작하지 않습니다. 시뮬·네임스페이스 변경 구성에서는 제어를 허용하지 않습니다.
보드 재시작 후에도 실행 중 세션을 표시하며, 브라우저를 닫으면 주행은 계속됩니다.
구동 준비/실패 사유와 자동 시작까지 걸린 시간을 상단에 표시합니다.

터미널에서는 Mac의 저장소 루트에서 같은 실행기를 사용할 수 있습니다.

Finder에서는 저장소 루트의 **`start_robot.command`를 더블클릭하면 구동**,
**`stop_robot.command`를 더블클릭하면 정지**합니다. Terminal에서 실행되며
보드나 채팅이 필요 없습니다. 이 파일들은 기존 실행기를 호출하므로 센서 수신이나
시간 점검에 실패하면 사유를 출력하고 출발하지 않습니다.

```bash
./robot.sh start     # 실제 주행; Ctrl-C 또는 다른 터미널의 stop으로 종료
./robot.sh stop
./robot.sh check     # 읽기 전용 시간·센서 점검
./robot.sh status
./robot.sh dry-run   # VM 연결도 하지 않고 명령 표시
```

각 주행은 기존 `run_policy.sh`의 콘솔·rosbag·업로드 재시도 경로를 사용합니다.
`startup.jsonl`에는 진입·점검 완료·launch·미션 시작·첫 움직임/이동·정지를 기록합니다.
격리 도메인 184의 가짜 센서/서비스 시험은 실제 출발 지연이나 마커 과제 성능의
측정이 아닙니다. 실제 구동은 별도 사용자 지시/버튼 클릭이 있어야 합니다.

## Gazebo 시뮬레이션

같은 화면을 Gazebo의 ROS 토픽에 연결하려면 `--sim`을 추가합니다. 이 모드는
`/clock`을 기준으로 TF 신선도를 판단합니다. Gazebo와 보드의 ROS 도메인,
RMW, Cyclone DDS 설정을 똑같이 맞추세요. 예를 들어 이 Mac의 Multipass
`osp-jazzy` VM에서 v20 시뮬을 실행했다면 별도 VM 터미널에서:

```bash
cd /home/ubuntu/simul
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/osp-v20/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
python3 dashboard/robot_board.py --sim --domain 48 --host 0.0.0.0 --port 8767
```

Mac에서 `http://<VM-IP>:8767`을 열면 현재 진행 중인 미션·지도·경로·RGB·LiDAR를
볼 수 있습니다. 시뮬이 끝나면 `시뮬 종료 · 마지막 화면`으로 표시됩니다. 새
Gazebo 실행으로 `/clock`이 되감기면 이전 실행의 경로와 상태를 지웁니다. v20 시뮬은
실물용 `scan_self_filtered`를 발행하지 않으므로 필터 항목은 적용되지 않았다고
표시합니다. 완료된 Gazebo 캠페인 영상과 평가 결과는 기존
`dashboard/server.py` 대시보드에서 확인합니다.

## 유선 연결에서 새 노드가 센서를 못 찾는 경우

2026-11-17 실물에서 eth0의 DDS 발견 그룹(239.255.0.1) 가입이 없고,
기존 보드는 수신하지만 새 VM 구독기는 메시지 0개인 상태를 확인했습니다.
로봇의 `osp-wired-discovery.service`가 유선 그룹 가입을 유지하도록 설치했습니다.
`scripts/keep_ros_discovery.py`는 발견 패킷을 수신·폐기하며 주행 명령은 보내지 않습니다.
사용자 서비스 자동 시작과 linger를 켰으며, 새 `./robot.sh check`가 통과했습니다.
재부팅 후 검증은 별도입니다. 로봇에서 상태는 아래처럼 확인합니다.

```bash
systemctl --user status osp-wired-discovery.service
loginctl show-user ubuntu --property=Linger
```

재설치는 스크립트를 `/home/ubuntu/keep_ros_discovery.py`, 서비스 파일을
`~/.config/systemd/user/osp-wired-discovery.service`에 복사한 뒤
`systemctl --user daemon-reload`, `systemctl --user enable --now osp-wired-discovery.service`,
`loginctl enable-linger ubuntu` 순서로 수행합니다.

실물 시작 점검은 카메라 촬영 시각 오차를 0.65초 이하로 제한합니다(미션 수락 한도
0.75초보다 엄격). 오래된 시각이 감지되면 주행 노드가 없고 실제 odom 정지가
확인될 때만 OAK-D stop/start를 한 번 수행하고 신선도를 재검증합니다. 정지 요청,
서비스 실패, 반복 지연은 출발 차단으로 종료하며 안전 신호를 무시하지 않습니다.
실물 rosbag은 OAK-D의 정책 입력인 `rgb/preview`를 보존하고 다른 대용량 영상은
제외합니다. 이 제외는 녹화 범위 변경이며 카메라 시각 오차의 원인으로 확정하지 않습니다.

로봇 bringup의 `50-osp-reliability.conf`는 chrony 오차가 10ms 이내로 안정된 뒤
드라이버를 초기화하게 하고, 종료 시 자식 센서 프로세스까지 10초 내 정리하도록
설정합니다. 설정 적용을 위한 현재 실행 중단은 하지 않습니다. 실제 재시작에서
직렬 초기화에 멈췄던 LiDAR는 단독 recovery 서비스로 정상 스캔을 복구했습니다.
압축 파생 토픽도 녹화에서 제외하여 RGB의 compressedDepth 변환을 유발하지 않습니다.

진행 중인 시작 RPC는 순간 카메라 지연만으로 취소하지 않고 미션 자신의
신선도 검사 응답을 처리합니다. 요청 전 신선도 검사, 범퍼/낭떠러지 즉시 중단,
응답 유실 시 재요청 금지는 유지합니다. 해당 회귀와 격리 ROS 검사 18개를 통과했습니다.

## 범퍼 접촉 뒤 복구

한 번의 범퍼 접촉은 감독기가 미션을 종료하지 않습니다. 기존 정책이 현재 목표를
취소하고, 신선한 후방 스캔과 TF로 후진 공간이 확인될 때 Nav2 BackUp으로
20cm를 0.08m/s로 후진합니다. 접촉한 목표는 제외하고 다시 계획합니다. 후방이
막혔거나 정보가 없으면 후진하지 않으며, 실제 범퍼/충돌 감시는 그대로 유지합니다.
접촉 횟수에 따른 종료 제한은 없습니다. 감독기는 10초 연속 접촉과 낭떠러지/스톨/바퀴 들림에만 안전 정지합니다.
시작 시 이미 눌린 범퍼는 여전히 출발을 거부합니다. 감독기 22개 및 기존 정책의
범퍼/후방 검사 10개가 통과했으며, 새 설정으로 실물 접촉을 고의 유발하지 않았습니다.
