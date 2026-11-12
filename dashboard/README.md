# OSP Robot Observatory

실제 로봇 시점, Gazebo 관찰 카메라, SLAM 지도와 LiDAR, 마커 진행, 로그와 완료 영상을
확인하는 **로컬 읽기 전용 대시보드**입니다. 기존 Tk 2D 재생 대시보드와 별개입니다.

```bash
cd /home/peter/simul
bash scripts/run_robot_dashboard.sh start
```

이 PC에서 <http://127.0.0.1:8765>를 엽니다. 사용자 systemd 서비스
`osp-robot-dashboard.service`가 채팅과 독립 실행되며 다음 로그인 때도 시작합니다.
PC가 꺼지면 서비스와 시뮬레이션도 중단됩니다. 외부 인터넷에 공개하지 않습니다.

```bash
bash scripts/run_robot_dashboard.sh status
bash scripts/run_robot_dashboard.sh restart
bash scripts/run_robot_dashboard.sh stop
journalctl --user -u osp-robot-dashboard.service -n 50
```

다른 포트는 `OSP_DASHBOARD_PORT=8766 bash scripts/run_robot_dashboard.sh restart`로 설정합니다.
설치 없이 전경 실행하려면 `bash scripts/run_robot_dashboard.sh run`을 사용합니다.
서버는 Python 표준 라이브러리와 프로젝트에 설치된 OpenCV를 사용합니다.
실시간 카메라 관측에는 기존 `/opt/ros/jazzy`의 rclpy·sensor_msgs·cv_bridge가 필요합니다.

## 표시하는 데이터

- 로봇 시점: 해당 실행의 ROS RGB를 벽시계 기준 최대1fps로 읽기 전용 구독합니다.
  10초 이상 수신이 없으면 녹화기의 실제 로봇 프레임으로 대체하고 저장 프레임이라고 표시합니다.
- 지도와 관찰 카메라: 기존 실제 Gazebo 녹화기의 `latest_frame.png`를 사용합니다.
  갱신은25개 녹화 표본마다로, 보통5초의 시뮬레이션 시간입니다. 현재 RTF에 따라 실제 시간은 더 깁니다.
- 상태와 로그:3초마다 읽습니다. ROS 상태 파일 자체는 대략15초의 실제 시간마다 갱신됩니다.
  45초 넘게 갱신되지 않은 실행은 경고를 표시합니다.
- 마커: 인식된 ID와 실시간 위치 추정으로 판정한1.5m 접근을 구분합니다.
  종료 후에는 `evaluation.json`의 독립 검증과 실제 최근접 거리·지도 좌표 오차를 표시합니다.
  마지막 실시간 파일이 미완료여도 최종 평가를 우선합니다.
- 공간: 원형·삼각형·사다리꼴 등 실제 시나리오 메타데이터의 공간 유형·크기와 형태별 진행 수를 표시합니다.
- 바닥 센서: 실제4개 cliff 센서 검사 결과와 원본 `floor_probe.json`을 표시합니다.
  시뮬레이션 환경 오류로 제외한 주행은 ‘환경 오류로 종료’로 구분하여 정책 성공으로 표시하지 않습니다.
- 설정 적용: 의도한 정책 파일·600초 제한·SLAM 설정·Nav2 응답 대기와 실제 ROS 값의
  일치 여부를 표시하고 `runtime_configuration.json`과 비교 결과를 다운로드합니다.
- 재방문: 새 후보의 odom 기반 주행/재방문 추정 거리와 원본 JSONL을 제공합니다.
  재방문에는 통로 통과 등 필요한 이동도 포함됩니다.
- 영상: 최종 평가와 `recording.json`이 모두 있는 완료 MP4만 제공합니다.
  녹화 중 MP4는 제공하지 않습니다. 검색·결과 필터·재생·구간 탐색·다운로드를 지원합니다.
- 파일: 선택한 실행의 이벤트JSONL·평가JSON·런타임CSV·로그를 다운로드합니다.
  임의 파일 경로를 요청하는 API는 없습니다.

‘실행 따라가기’를 켜면 종료 후 다음 활성 실행과 새 활성 캠페인으로 이동합니다.
과거 실험을 직접 선택하면 자동 따라가기를 해제합니다.
과거 영상의 당시 성공 판정과 현재1.5m 기준 통과 여부는 다를 수 있습니다.

서버는 `training_robust/native90/*/state.json`과 영상 인덱스를 읽습니다.
카메라 관측기는 실행ID별 캐시를 별도로 사용하고 ROS domain 재사용 시 새 관측기로 교체합니다.
시나리오/정답 좌표를 정책에 전달하거나 로봇 명령을 발행하지 않습니다.
고정된 runtime 소스와 훈련 상태 파일도 수정하지 않습니다.
전체 서비스는 MemoryHigh512MiB, MemoryMax768MiB, CPUQuota60%, Nice10으로 제한합니다.
카메라 없는 점검은 `python3 dashboard/server.py --no-cameras --port 8766`로 실행합니다.

## 검증

```bash
python3 -m pytest dashboard/test_server.py -q
```

완료 평가 우선순위,1.5m 성공 기준, 중단된 실행, 미완료 영상 제외,
HTTP 구간 탐색/HEAD/416, 경로 침범 거부, 로그 크기 제한을 검사합니다.
실제 브라우저에서 카메라·지도, 완료 결과, 영상 재생과 로그 화면을 확인했습니다.
GitHub Release 영상 업로드는 별도 `scripts/publish_simulation_videos.py --upload` 작업입니다.
대시보드 자체가 GitHub로 자동 업로드하지는 않습니다.
