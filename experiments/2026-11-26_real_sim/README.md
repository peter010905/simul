# 실물 로그 기반 시뮬 보정 1차

2026-11-26 KST 실물 기록 8회를 조사했다. 기록 ID는 UTC이므로 `20261125T...`로 시작한다. 기존 정책, 실물 카메라, 충돌 감시와 실제 주행은 변경하지 않았다. 이번 결과는 센서 조건 일치와 장애물 재관측 검증이며, 마커 탐색 성공률 개선을 입증한 실물/시뮬 주행 비교는 아니다.

## 확인된 차이와 적용 범위

| 항목 | 기존 시험장 Gazebo에서 측정 | 실물 8회에서 측정 | 보정판 Gazebo에서 측정 |
|---|---:|---:|---:|
| RGB 입력 | 640×480, 5fps | 250×250, 약 10fps | 250×250, 10fps |
| LiDAR 빔 수 | 640 | 720 | 720 |
| LiDAR 주기 | 10Hz | 중앙값 약 8.11Hz | 약 8.06Hz |
| 원본 LiDAR 최대 범위 | 12m | 12m | 12m |
| SLAM 최대 사용 거리 | 부모 설정 5m | 12m 3회 / 5m 5회 | 5m와 12m 별도 사본 |

실물 해상도는 폴더 이름이 아닌 JPEG를 직접 복호화한 크기와 `/mission/processed_camera`의 CameraInfo로 확인했다. 검출기는 영상과 CameraInfo 크기가 다르면 프레임을 처리하지 않으므로, 처리 완료 신호 역시 실제 250×250 입력의 근거가 된다. 기존 640×480·5fps는 목표 카메라 조건의 민감도 실험으로 보존한다. 실물의 현재 카메라 설정을 이번 작업에서 바꾸지는 않았다.

원본 `/scan`의 12m 범위와 SLAM의 5m 사용 한계를 분리했다. 5m 프로필도 시뮬 LiDAR 자체를 5m로 자르지 않는다. 센서 조건은 6개 실행에서 추정하고 12m·5m 각 1개 실행을 통째로 보류해 검증했다. 보류 실행에서 LiDAR 중앙 주기 오차는 각각 약 0.006%, 0.129%였다. 전체 실행 사이에 지도 셀이나 장애물 위치를 합치지 않았다.

정상 수신 중의 주기와 기록 중단을 구분했다. 예를 들어 `195426` 기록은 rosbag 종료 전 약 239.5초 동안 scan·odom·processed-camera 메시지가 더 기록되지 않았다. 기록 경로, 노드 종료, 네트워크 단절 중 어느 원인인지 이 통계만으로 확정하지 않는다. 현재 보정판은 정상 수신 조건을 재현하며, 이런 중단이나 실제 부하를 자동으로 재현하지 않는다.

## 반복 관측 장애물

같은 SLAM 이미지가 반복 발행된 횟수는 장애물 관측으로 세지 않는다. 최종 지도에서 점유된 셀을 과거 `/scan_self_filtered`의 실제 유효 반사와 비교한다. 각 스캔의 노출 시점에 기록된 `map→odom→...→rplidar_link` 변환을 사용한다. 변환이 없거나 오래된 시점은 제외한다.

- 관측 간 최소 2초, 시점 이동 25cm 또는 회전 15°.
- 최소 3개 시점의 재관측과 최초·최종 관측 간 최소 6초.
- 지도 셀과 반사 거리 차이 12cm 이내. 이후 유효 광선이 해당 위치를 지나간 빈 공간 증거가 많으면 제외.
- 무효 반사·미지 영역·검증 실패는 빈 공간으로 변환하지 않는다.
- 이 임계값은 재구성 후보 추출용이다. 정지한 사람, SLAM 왜곡까지 완전히 제거하는 의미 분류기는 아니다.

| 실물 기록 | SLAM | 유효한 서로 다른 시점 | 점유 셀 | 반복 지지 셀 | 상태 |
|---|---:|---:|---:|---:|---|
| `195126-1cce5b818a` | 5m | 76 | 2,346 | 1,021 | 재관측 근거 있음 |
| `195426-a84b4e02f7` | 5m | 173 | 5,685 | 3,111 | 재관측 근거 있음 |
| `184804-71cb633767` | 5m | 0 | 6,629 | 판정 불가 | 시각별 TF 연결 부족 |
| `174857-2f02f6e167` | 12m | 0 | 12,807 | 판정 불가 | 시각별 TF 연결 부족 |

두 성공 사례는 별도 지도의 반복 지지를 확인한 결과다. 같은 실제 물체임을 지도 사이에서 정합해 확인한 결과는 아니다. 추출한 구조는 검토용으로 남겼으며 기존 시험장에 위치를 맞추지 않고 덮어씌우지 않았다. 2D SLAM만으로 장애물의 전체 높이, 모양, 재질을 복원하지 않았다.

## 책상 다리 수

검사한 시험장 SDF의 `demo_furniture`에는 책상 상판 **14개**, 이름이 명시된 다리 충돌 요소 **56개**가 있다. `furniture.json`에 모델 해시와 전체 이름을 저장했다. 의자 다리는 포함하지 않는다. 실물 지도에서 보이는 작은 점유 덩어리는 사람·노이즈·의자 다리와 구분되지 않으므로 **실물 책상 다리 총수는 이번 자료만으로 확정하지 않았다.**

## 검증과 실행

`comparison.json`은 동일한 원본 시험장 배치에서 각각 20초 이상의 시뮬 센서 데이터를 받은 결과다. 새 프로필의 크기·주기·빔 수·원본 범위 검사가 모두 통과했다. Gazebo 시간 간격의 양자화 때문에 요청 8.11Hz는 측정 약 8.06Hz였다. 12개 집중 테스트도 통과했다. 처음 실행 중 발생한 DDS participant 수 부족, 상위 namespace 누락, IR 센서까지 포함되던 생성기 선택 문제를 수정했다. 실패 실행은 VM에 보존했다.

이번 native 검사는 SLAM·Nav2·마커 탐색을 실행한 완주 시험이 아니다. SLAM의 5m/12m 설정은 각 사본의 YAML과 부모/정책/안전 설정 보존 해시로 확인했다. 실제 실행 중인 SLAM 노드의 거리별 지도 품질은 추가 비교가 필요하다.

시뮬 전용 사본:

- `/home/ubuntu/real_sim_measured5_r2_20261126_ws`
- `/home/ubuntu/real_sim_measured12_20261126_ws`

활성화는 해당 사본을 source한 시뮬 프로세스에만 `OSP_REAL_SIM_PROFILE=<workspace>/real_sim_profile.json`을 지정한다. 173·174·175·185의 격리된 시뮬 도메인에서만 허용한다. 실물 보드 선택지는 바꾸지 않았다. 근거리 IR 센서, 로봇 충돌 형상, Nav2 설정과 정책 본체는 부모와 동일하다.

```bash
# Jazzy VM, offline audit (publish 없음)
source /opt/ros/jazzy/setup.bash
python3 audit.py --root ~/simul/run_logs/robot --output audit.json <run-id>...
python3 profile.py audit.json profiles
python3 persistent_scene.py ~/simul/run_logs/robot/<run-id> ~/real-sim-evidence/<run-id>
python3 -m unittest discover -s . -p test_real_sim.py -v

# 새 사본 생성. 기존 사본을 덮어쓰지 않음.
python3 install.py <parent-workspace> <new-sim-workspace> profiles/measured-slam5.json
cd <new-sim-workspace>
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash

# 실제 시험장 fixture로 passive native 센서 검사. 주행·mission 요청 없음.
python3 <experiment-dir>/run_probe.py <new-sim-workspace> <exam-fixture> <new-output-dir> \
  --profile <new-sim-workspace>/real_sim_profile.json
```

남은 주요 차이는 광학 왜곡·FOV 보정, 회전 중 스캔의 시각 왜곡, 유리의 반사/투과, 노트북 중량에 따른 미끄러짐·가감속, 실제 접촉 형상, 부하 중 센서 지연이다. 장면에 따라 달라지는 무반사 비율을 임의의 확률로 전체 빔에서 삭제하거나, 휠 odom을 참값으로 간주해 마찰을 추정하지 않았다.
