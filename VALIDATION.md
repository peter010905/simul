# 실행 검증 기록 — 2026-11-04

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic, TurtleBot4 Standard.

- `colcon build --symlink-install`: 성공.
- `colcon test --packages-select osp_mission`: pytest 4개 통과.
- Async 모드: LiDAR 실제 거리값과 지도 생성 확인.
- Sync + session=continue: 저장된 map.posegraph/map.data 읽기 및 Nav2 활성화 확인.
- `/mission/start`: success=true. 자동 undock과 odom 이동 확인.
- `NavigateToPose`: status=4(SUCCEEDED), error_code=0.
- 마커 시험: 시작 world=(6,1.2,-pi/2), ID 7의 정답 map=(1.05,0).
- 검출 좌표 map=(1.047781,0.002509), XY 오차 약 0.0034m. 한 배치에서의 결과이며 일반 정확도 보장값이 아님.
- 10초 시뮬레이션 제한 후 timeout 정지 및 저장 성공.
- 결과: `results/20261104_211132/` — PGM/YAML, posegraph/data, markers.csv, result.json.
- 지도 이미지: 395×215, 해상도 0.05m, 원점 (-10.100,-6.846), occupied_thresh=0.65, free_thresh=0.196.
- 바닥 패턴의 오검출을 확인하여 작은 패턴과 수평면 제외 규칙 및 회귀 검증 추가.

남은 성능 검증: 세 가지 미공개 배치에서 600초 완주, 충돌 횟수 계측, 실물 LAN 및 실제 센서 정렬, 큰 loop closure에서 마커 오차 측정.


## 장애물 숨김 과제와 이동 정체 수정

- 세 마커 모두 서로 다른 장애물의 시작점 반대 면에 부착하는 배치 생성기로 전환. 시작점에서 360도 회전해도 마커를 찾지 못하는 조건을 검사.
- 벽면 마커 학습은 중단하고 별도 기록으로 보존. `scenarios_hidden` 200개 / `training_hidden` 신규 학습 및 감독 프로세스 실행.
- 실제 SLAM 지도의 sparse scan 구멍 때문에 후보 0개로 회전만 반복하던 문제 재현. 실제 점유 장애물 기준으로 여유 거리를 계산하고, 목표 지점 주변 관측 밀도를 검사하도록 수정. 같은 지도에서 후보 16개 확인.
- Gazebo에서 Nav2 목표 (-0.20,1.98), (0.99,4.09), (-0.73,6.51)m에 연속 도착한 로그 확인.
- 8초 관측 중 odom XY가 (-1.0422,1.9539) → (-1.2201,2.4594)m, 최대 선속도 0.26m/s, 접촉 사건 0회.
- pytest / colcon test: 11개 통과. 전체 600초 세 마커 완료와 실물 검증은 아직 통과하지 않음.
