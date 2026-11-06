# 센서·격자 보정과 다양한 배치 검증 — 2026-11-05

## 장애물 16개 실제 성공과 다음 후보

`native90/motion_v2_drive_ready_mixed16`은 **356.447초, 정확한 세 ID, 충돌 0회**로 완료했다. 최종 좌표 오차는 ID 984=0.0117m, 199=0.0444m, 352=0.0533m이며, 세 마커 모두 2m 안쪽까지 접근했다. `artifact_audit.json`에서 지도·pose graph·CSV와 실제 Gazebo/RGB/SLAM 영상 1,783프레임의 첫/마지막 디코딩을 확인했다. 실행 중 사용한 소스 119개도 보존본과 일치한다. 영상은 `native90/motion_v2_drive_ready_mixed16/episodes/8600007/video/mission.mp4`다. **개발 배치 한 개의 성공이며 90% 검증은 아니다.**

이 후보는 기존 면별 관측 기억을 사용하고, 관측 시각의 지도 좌표 저장·구동 준비 점검과 전진 0.30m/s·스캔 회전 0.85rad/s 설정을 함께 적용했다. 별도 후보 `persistent_map_v4_screen3`의 16/24/32개 실제 캠페인은 계속 진행 중이다. 기존 실패 결과를 소급해서 성공으로 바꾸지 않았다.

첫 짧은 이동에서는 목표에서 약 0.34m 떨어진 채 약 42초를 사용했다. 최종 방향 정렬 비용을 적용하는 거리 0.5m를 위치 허용 오차와 같은 0.25m로 줄이는 비교를 실제 Gazebo의 빈 공간에서 분리 시험한다. 이 진단은 전체 자율 임무 성공률에 포함하지 않는다. `runtime_panorama_v5_staging`은 각 관측 지점에서 360도 스캔하는 후보이며 테스트 155개와 빌드를 통과했다. `runtime_focus_v6_staging`은 카메라에서 실제로 수락된 1–2회 관측을 다음 의사결정에서 가까운 안전 지점에서 다시 확인한다. ID당 최대 두 번·30초 간격으로 제한하며 3프레임 확정과 18픽셀 기준은 유지한다. 이 후보의 테스트 169개와 빌드가 통과했지만 두 후보 모두 실제 임무 성능은 아직 미검증이다.

짧은 이동 비교에서는 기존 정렬 거리 0.5m가 3.618초, 변경한 0.25m가 20.074초였다. 한 쌍의 결과이므로 일반적인 인과 효과는 확정하지 않지만, 이번 후보에는 0.25m를 채택하지 않는다. 중간의 Nav2 서비스 요청 실패로 시작하지 못한 시도도 별도 보존했다. `panorama_v5_screen2`는 기존 0.5m를 유지한 채 16/32개 실제 시험을 시작했다. `persistent_map_v4_screen3`의 첫 16개 배치는 600.12초에 1개 확정·충돌 0회로 실패했고, 24개 배치로 진행했다.

실제 실패 기록에서 미확정 ID 352를 읽은 뒤 다음 관측지 선택까지 약 24.5초가 지나 5.84m 떨어지는 사례를 확인했다. 대기 중인 재확인 후보는 수락된 RGB 관측을 받으면 주행을 ID당 한 번, 전체 최대 여섯 번 중단하고 1초 동안 추가 프레임을 확인하도록 보완했다. 이때 읽은 카메라의 관측 시각 지도 위치도 함께 전달한다. 마커 확정 기준과 18픽셀 기준은 그대로다. 같은 실패 기록의 놓친 마커 면은 끝까지 미관측 상태였으므로, 미리 관측 완료로 지웠다는 가설은 근거가 없었다. 검사 근거는 `native_diagnostics/marker_hint_delay_v4.json`, `persistent_v4_surface_claims.json`, `persistent_v4_view_opportunities.json`이다. 새 후보의 실제 임무 성능은 아직 미검증이다.

최종 판정은 `training_robust/audit_final_native.py`로 하나 이상의 실제 캠페인을 합산한다. 예약한 정확한 40개 배치, 중복 없는 40회 시도, 동일 정책·소스 보존본, 실제 센서 수신·지도·좌표·영상과 최소 36개 성공을 확인한다. 개발 배치 한 개의 성공은 이 판정에서 거부됨을 확인했다.

후속 실제 결과에서 `persistent_map_v4_screen3`의 24개 배치는 600.305초에 2개 발견·충돌 0회, `panorama_v5_screen2`의 16개 배치는 600.0초에 2개 발견·충돌 0회로 끝났다. 저장 좌표는 각각 최대 0.0893m와 0.1061m 오차였지만 세 ID 조건을 충족하지 못했다. 원래 실패 판정과 영상·지도를 감사했다. 두 캠페인은 마지막 32개 배치를 계속 실행한다. 낮은 12픽셀 기준의 읽기 전용 후반 영상 비교 1,261/1,421프레임에서도 새 확정 ID는 없었다. 16개 360도 관측 실행에서는 놓친 ID 984가 전체 원판독 기록에도 디코딩되지 않았다.

따라서 아직 실행하지 않은 `runtime_focus_v6_staging`은 **356.447초 성공 후보의 일반 면별 기억·적응형 부분/전체 스캔·동일 주행 설정**으로 기반을 맞추고, 실제 RGB 마커 즉시 재확인 기능을 추가했다. 이전 지속 기억·강제 전체 스캔 기반 소스는 `native90/focus_v6_persistent_pre_native.tar.gz`에 보존했다. 변경 후 전체 테스트 169개와 빌드를 통과했다. 준비된 `marker_focus_v6_screen3`은 16/24/32개 개발 배치를 사용하며 아직 실제 시작 전이다.

## 최신 실제 Gazebo 결과: 첫 완주와 다중 배치 시험

2026-11-05 통신 권한, 실제 RGB/LiDAR/IMU 수신과 주행을 확인했다. 아래의 소켓 차단·미검증 설명은 당시의 과거 기록이다.

`native90/development_imu_v1/runtime_probes/20261104T230250_371315Z/evaluation.json`에서 장애물 12개·장애물 뒷면 마커 세 개의 실제 임무가 **238.9초, 충돌 0회**로 성공했다. ID 634/670/744가 모두 일치하고 좌표 오차는 각각 2.7/2.8/5.0cm다. `map.pgm`, `map.yaml`, `map.posegraph`, `map.data`, `markers.csv`가 모두 저장됐다. 정답 위치는 평가기만 구독한다. 실제 최단 접근 거리는 3.74/1.45/0.97m로, PDF의 2m 접근 가점은 세 개 중 두 개에 해당한다.

원인은 세 층으로 나뉘었다. 이미지 노드의 제한된 DDS 설정으로 5Hz 카메라 수신을 복구했고, 현재 GPU 드라이버의 누락된 LiDAR 방향은 해당 실행에만 소프트웨어 렌더링을 적용해 수정했다. 이후 실제 회전량이 바퀴 기반 추정치보다 약 3.2% 컸던 문제는 바퀴의 선속도와 IMU 각속도를 EKF로 결합해 수정했다. 절대 시뮬레이터 위치·자세는 추정기나 정책에 넣지 않았다. PDF의 지정 출발 자세에서 바로 시작하며, 도크 출발은 별도 옵션 `start_docked:=true`다.

이번 실행의 카메라 5Hz, LiDAR 10Hz, odom 약 62Hz이며 실시간 계수는 0.619다. 영상은 239.0초/1,195프레임으로 처음과 마지막 프레임의 디코딩을 확인했다. 원본 영상은 다음 위치에 있다.

[Gazebo 완주 영상](/home/peter/simul/training_robust/native90/development_imu_v1/runtime_probes/20261104T230250_371315Z/video/mission.mp4)

동일한 보정 설정과 가중치를 고정한 `native90/development_imu_screen4`의 장애물 8/16/24/32개 및 walls/mixed/obstacles/hidden_obstacles 시험은 **1/4 성공**으로 끝났다. 8개 배치는 117.2초에 성공했고 나머지는 600초에 두 개 발견으로 종료됐다. 충돌은 전부 0회지만 32개 배치의 좌표 두 개도 0.5m 기준을 넘었다. 첫 완주와 추가 개발 시험은 최종 성공률 검증이 아니다. 예약한 네이티브 독립 시험 40개는 아직 평가하지 않았으며, 36개 이상 성공해야 90% 기준에 도달한다. 완료 조건은 600초 이내 정확한 세 ID와 0.5m 이내 좌표, 충돌 0회, 필수 산출물이다.

접촉 시 현재 Nav2 동작을 취소하고, LiDAR 설치 각도를 base_link로 변환해 실제 후진 방향의 장애물을 검사하는 복구도 추가했다. 이 변경은 첫 완주 후 추가 개발 시험부터 적용한다. 테스트 135개 및 빌드가 통과했으나, 실제 접촉 복구의 효과는 아직 따로 검증되지 않았다.

## 실제 관측 기록의 유실 진단과 비교 후보

16개 장애물 개발 배치의 실제 지도·LiDAR·처리된 카메라 시각을 184프레임 저장해 재생했다. 기존 면별 관측 메모리는 SLAM 원점과 격자가 조금씩 변할 때 확정 기록을 반복해서 지웠다. 이 구간에서 새로 확정한 기록은 누적 349개, 지도 갱신으로 잃은 기록은 218개, 마지막에 남은 기록은 131개였다. 입력과 집계는 `training_robust/native_diagnostics/edge_coverage_retention16`에 있다.

`runtime_v3_staging` 후보는 실제 카메라 세 프레임으로 확인한 면을 미터 단위 위치와 법선 방향으로 보존한다. 지도 갱신 시 7.5cm 이내이고 같은 방향인 가장 가까운 면 하나에만 연결한다. 재투영은 새 기준점을 만들지 않으므로 반복 갱신으로 관측 위치가 밀려가지 않으며, 장애물 뒷면에 앞면 기록을 복사하지 않는다. 같은 입력에서 기록 손실은 18개, 마지막 기록은 166개였다. 이는 기록 보존 검사이며 임무 성공률 향상을 입증하는 결과는 아니다.

대각선 얇은 벽 검사까지 포함한 전체 테스트 152개와 빌드가 통과했다. 실제 Gazebo 비교는 `native90/persistent_edge_v3_mixed16`에서 시작했다. 속도는 기존과 동일하다. 기존 면별 후보 `edge_runtime_v2`는 600.306초에 2개 발견, 충돌 0회로 끝났고 마커 좌표 최대 오차도 0.640m여서 성공 기준에 미달했다.

완료한 실제 실행 다섯 개의 마커 기록 12개를 비교하니, 마지막 시점의 map←odom 변환을 과거 관측에 적용하는 방법이 오래된 좌표를 이동시키고 있었다. 최신 실패 실행의 0.381/0.640m 오차는 관측 당시 지도 좌표를 유지한 재생에서는 0.050/0.086m였다. 전체 12개의 재생 오차는 최대 0.122m였지만, 이 계산으로 기존 실패 판정을 바꾸지는 않는다. 근거는 `native_diagnostics/marker_anchor_comparison.json`이다.

별도 `native90/motion_v1_mixed16` 후보는 기존 면별 메모리를 유지하고 관측 시점의 지도 좌표를 저장하도록 수정했다. 동시에 전진 상한 0.26→0.30m/s, 관측 회전 상한 0.55→0.85rad/s, 회전 가속도 1.2→1.8rad/s²를 비교한다. 따라서 속도 하나만의 효과를 분리하는 실험은 아니다. 물리 제어기의 0.46m/s·1.9rad/s 제한과 카메라·가중치는 바꾸지 않는다. 좌표 수정의 회귀 검사를 포함해 테스트 147개와 빌드가 통과했으며 실제 실행은 대기 중이다. 두 후보 모두 기본 실행에 채택하지 않았다.

## 관측 기록 보존 후보의 실제 완주와 구동 시작 오류

`native90/persistent_edge_v3_mixed16`은 기존 두 실행이 놓쳤던 ID 352까지 포함해 **358.113초에 세 ID를 모두 찾았고 충돌은 0회**였다. 그러나 최종 좌표 오차가 984=0.618m, 352=0.816m, 199=0.121m여서 성공 판정은 false다. 영상 1,793프레임/358.6초와 지도·CSV를 감사했으며 오차 기준 외 산출물은 정상이다. 관측 당시 지도 좌표의 마지막 30개 중앙값을 사용하는 재생에서는 각각 0.053/0.020/0.119m였다. 원래 결과를 성공으로 소급 변경하지 않았다. 기록 보존과 좌표 수정, 구동 시작 점검을 합친 `native90/persistent_map_v4_screen3`은 16/24/32개 배치를 실제 실행하며, 전체 테스트 154개와 빌드가 통과했다.

속도 비교 첫 시도 `native90/motion_v1_mixed16`은 바퀴 제어기가 로드되지 않은 상태에서 임무가 시작됐다. 읽기 전용 ListControllers 호출로 joint_state_broadcaster만 active임을 확인했고, 0.85rad/s 명령에도 실제 회전은 0이었다. 운영자가 종료한 이 시도는 유효한 속도 비교가 아니다. 설치된 spawner 코드는 매개변수 설정 응답을 기한 없이 기다릴 수 있었고, 해당 로그에는 응답 전송 실패가 있었다. Gazebo가 이미 전달한 제어기 설정 파일을 spawner가 중복 설정하지 않도록 수정했다. 이제 Gazebo의 임무 시작은 diffdrive_controller active와 신선한 wheel/odom을 모두 확인하며, RPC 응답 소실도 기한 내 재시도한다. 실제 로봇 실행에는 Gazebo 전용 구동 점검을 기본 적용하지 않는다.

기본 코드의 관련 테스트 149개와 빌드가 통과했다. 재시도 `native90/motion_v2_drive_ready_mixed16`에서는 바퀴 제어기 활성화와 기존 휠 보정값을 확인했고 실제 제자리 회전 중앙값 약 0.877rad/s를 측정했다. 시험은 진행 중이며 성능 개선으로 확정하지 않았다. 기본 빌드가 보관한 실험 패키지를 중복 탐색하지 않도록 `training_robust/COLCON_IGNORE`도 추가했다. 12픽셀 그림자 디코더의 1,043프레임에서는 기존 18픽셀 디코더보다 새 마커 ID를 더 얻지 못해 생산 설정은 18픽셀을 유지한다.

## 호스트 실행의 도크 이탈 취소 수정과 실제 영상 녹화

호스트 실행 `gazebo_runtime_calibration/runtime_probes/20261104T211510_211783Z`에서는 실제 Gazebo 센서가 연결됐다. 임무는 **2.759 시뮬레이션 초에 `undock_failed`로 종료**됐다. `launch.log`에는 undock 시작 뒤 취소 요청이 기록되어 있다. 이전의 도구 환경 소켓 차단 기록과 구분한다. 이번 호스트 실행의 실시간 계수는 약 0.356이며, 카메라 3개 표본만으로 지속적인 처리 속도나 탐색 성능을 판단하지 않는다. 이동 속도 표본은 도크 이탈 중의 값이고 회전 표본은 0개다.

기존 임무 코드는 카메라 관측이 실제 시간 2초 동안 갱신되지 않으면 도크 이탈도 취소했다. 평가기 기록에서 마지막 카메라는 sim 7.8초, 취소 시 정지 명령은 sim 9.0초다. 두 메시지의 수신 간격은 실제 시간 3.17초지만 시뮬레이션 시간은 1.2초다. 당시 취소 사유와 action status는 저장되지 않아 어느 준비 조건이 최초로 실패했는지는 확정할 수 없다. 다음 변경으로 이 취소 경로를 수정했다.

- 센서 신선도는 메시지 시각과 ROS 시계를 비교한다. 시뮬레이션에서는 물리 시간, 실물에서는 ROS 시스템 시계를 사용한다.
- Create 3의 도크 이탈 동작에는 카메라·지도·Nav2 준비 상태를 적용하지 않는다. LiDAR 단절과 65초 제한은 유지한다. 성공 상태와 `is_docked == false`를 모두 확인한 뒤 탐색한다.
- 디코딩된 카메라의 생존 확인과 표면 관측 누적을 분리했다. 정확한 시각의 TF가 늦으면 관측을 재시도하며, TF 없이 표면을 관측했다고 처리하지 않는다.
- `result.json`에 undock action 상태, 도킹 여부, 취소 사유, 경과 시간을 기록한다.

같은 실패 배치를 재검사하면서 실제 카메라·지도를 녹화하려면 호스트 터미널에서 실행한다.

```bash
bash /home/peter/simul/run_gazebo_runtime_probe.sh --record-video
```

사용자가 요청한 **장애물 12개 + 장애물 뒷면 마커 3개** 배치의 영상은 다음 명령으로 녹화한다. 준비된 20개 배치 중 index 16이며, 다른 배치는 `--scene-index 0`부터 `19`까지 선택한다.

```bash
bash /home/peter/simul/run_gazebo_video.sh
```

실행마다 새 결과 폴더의 `video/mission.mp4`에 Gazebo 관찰 카메라, 로봇 RGB, 실제 `/map`·`/scan`·TF·확정 마커를 합성한다. 로봇이 모르는 공간은 지도에서 미탐색 상태로 남는다. 관찰 카메라는 녹화에만 사용하며 탐색기에 정답 위치를 전달하지 않는다. 1280×720 H.264, 시뮬레이션 시간 1배속이다. 입력 영상이 오래되면 STALE을 표시하고 프레임 지연은 직전 화면 유지로 표현하며 보간하지 않는다. `frames.csv`와 `recording.json`에 원본 시각, 유지 프레임 수, 입력 수신 여부를 기록한다. 실제 카메라 영상이 한 장도 없으면 동영상이 있다고 보고하지 않는다.

녹화용 카메라는 [Gazebo Harmonic 공식 카메라 예제](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/camera_sensor.sdf)의 센서 형식을 사용하며 기본 벤치에서는 꺼져 있다. 녹화가 렌더링 부하를 늘리므로 녹화 실행의 실제 시간 속도를 녹화하지 않은 벤치와 동일시하지 않는다.

검증: 전체 단위 테스트 **110개**, ROS 빌드, 녹화 world의 SDF 유효성 검사 통과. H.264 인코딩·재생과 시간 간격 보존은 합성 테스트 프레임으로 확인했다. **수정 후 실제 주행 및 영상 수신은 아직 검증하지 못했다.** 현재 도구 환경에서는 UDP 소켓 생성이 거부되어 호스트 재실행이 필요하다. 기존 실패 실행에는 영상이 없어 복원할 수 없다. 아래 절의 학습·실행 수치는 이전 단계의 기록이다.

## 관측 방향 정렬 비교와 실제 실행의 병목

학습 배치에서 실패한 대칭 후보 주행 **16개를 정확히 재현**했다. 놓친 마커 21개 중 20개에는 기하학적으로 볼 가능성이 있는 관측 후보가 있었다. 이 필터는 디코더의 세 프레임 확정이나 실제 도착을 보장하지 않는 낙관적인 진단이다. 그런 후보가 있었던 219개 의사결정에서 선택된 관측 면의 절반 이상이 긴 벽에 해당한 경우는 83개였다. 연결된 면의 길이를 1.5m로 제한하는 반사실적 점수는 놓친 마커 후보가 1위인 경우를 1개에서 3개로만 늘렸다. **긴 벽의 점수 편향을 지배적인 실패 원인으로 단정할 근거는 부족했다.** 정답 마커 좌표는 진단에만 사용했으며 실제 선택에는 영향을 주지 않았다. 기록은 `face_priority_diagnostic/completed.json`에 있다.

그다음 연속적인 관측 각도 범위를 유지하면서, 360도 스캔 전 정렬을 생략하고 부분 스캔을 반대 방향으로도 수행하는 실행 방식을 비교했다. 센서·주행 속도·회전 속도·학습 가중치는 그대로 두었다. 별도 학습 배치 80개에 네 조합을 적용해 **320회**를 완료했다.

| 동일 학습 배치 80개 | endpoint 기본 | endpoint 정렬 단축 | 대칭 후보 기본 | 대칭 후보 정렬 단축 |
|---|---:|---:|---:|---:|
| 세 마커 발견 | 67/80 | 63/80 | 63/80 | 61/80 |
| 뒷면 마커 조건 | 14/20 | 10/20 | 12/20 | 9/20 |
| 실패를 포함한 평균 임무 시간 | 422.6초 | 432.1초 | 415.3초 | 422.3초 |
| 관측 전 정렬 시간 | 15.4초 | 6.4초 | 9.7초 | 3.7초 |
| 접촉 | 0 | 0 | 0 | 0 |

정렬 시간은 줄었지만 전체 완료율과 평균 임무 시간이 악화됐다. endpoint에서는 실패 3개가 성공으로 바뀌는 동안 성공 7개가 실패로 바뀌었고, 대칭 후보에서는 각각 3개·5개였다. **두 후보 모두 채택하지 않았다.** 연속 관측 범위가 같아도 프레임 순서·다음 출발 방향·후속 탐색 경로까지 같지는 않다. 이 수치는 학습 묶음의 결과이며 아래의 개발 검증 160개나 과거 독립 시험과 직접 비교하지 않는다. 이번에는 개발 검증이나 예약 독립 시험을 새로 사용하지 않았다.

![관측 방향 정렬 비교](training_robust/execution_ablation/comparison.png)

`optimize_scan_alignment`는 2D 실험 옵션이며 기본값은 false다. ROS 기본 정책과 가중치는 바꾸지 않았다. 기본 endpoint의 첫 40개 주행은 이전 평가와 모든 기존 지표가 정확히 일치했다. 전체 코드 테스트 **97개**와 ROS 빌드가 통과했다.

실제 센서 처리 주기와 달성 속도를 확인하려고 Gazebo를 다시 실행했다. 첫 수동 명령은 `install/setup.bash` 누락으로 패키지를 찾지 못했으며, 이는 권한 차단의 증거로 사용하지 않는다. 빌드와 환경 설정을 포함한 `run_gazebo_runtime_probe.sh`로 고친 두 번째 시도에서는 패키지를 정상적으로 찾았지만 UDP 소켓 생성이 `Operation not permitted`로 거부되고 ROS participant 생성이 실패했다. 8.3초 후 시작 실패로 종료됐으며 camera·odom·clock 측정은 모두 0개다. 완료한 Gazebo 임무는 여전히 0개다. 정확한 로그는 `gazebo_runtime_calibration/runtime_probes/20261104T204355_004287Z/launch.log`와 `evaluation.json`에 있다.

이제 추가 모델 조정 전에 실제 실행 자료가 필요하다. 장애물 수·마커 부착 방식별로 균형 잡힌 Gazebo 배치 20개를 준비했고, 아래 명령은 그중 **첫 배치 1개만** 실행한다. ROS 통신과 렌더링이 가능한 호스트 터미널에서 실행한다.

```bash
bash /home/peter/simul/run_gazebo_runtime_probe.sh
```

이 명령은 빌드·환경 설정 후 별도 ROS domain 84에서 600초 시뮬레이션 임무를 시도한다. 결과와 실제 카메라 주기·회전/이동 속도·실시간 계수는 `training_robust/gazebo_runtime_calibration/runtime_probes/실행시각/`에 저장한다. 기존 실패 로그를 덮어쓰지 않는다. 이는 모델 보정을 위한 한 번의 실행이며 강건성 검증 완료를 뜻하지 않는다. 현재 제한된 도구 실행 환경에서는 이 측정 단계가 차단되어 있다.

## 공간 후보의 좌표 편향과 추가 검증

이번 단계는 **서로 다른 학습 배치 144개에서 총 860회**를 완료했다. 구조 비교 440회, 회전 모델 비교 60회, 가중치 탐색·대조 평가 360회다. 별도의 개발 검증 160회도 완료했다. 중단된 미완료 주행과 회귀 재현은 이 수치에 포함하지 않는다. 모든 완료 실험의 2D 접촉은 0회였지만, 강건하게 우수한 정책을 얻지는 못했다. 기본 정책과 가중치는 유지했다. 집계는 `training_robust/geometry_study_summary.json`에 있다.

기존 면별 탐색은 지도 원점에 고정된 1m 타일에서 관측 후보를 뽑았다. 합성 지도 하나를 90도 돌리는 검사에서 원래 후보 133개와 회전 후 후보 135개 중 같은 물리 위치는 20개뿐이었다. 관측된 면의 중심선에서 후보를 뽑거나, 관측 공간의 범위에 맞춰 타일을 대칭적으로 배치하는 방법을 비교했다. 이는 후보 생성의 좌표 편향 검사이며 전체 정책의 회전 불변성을 증명한 것은 아니다.

학습 배치 40개(8500100–8500139)에서 기본 endpoint 31/40, 기존 edge 33/40, 직선 면 후보 26/40, 작은 곡면도 포함한 면 후보 31/40, 대칭 공간 후보 `symmetric_edge` 35/40이었다. 대칭 후보의 접촉은 0회, 실패를 포함한 평균 시간은 399.4초였다. 직선만 다루는 방식은 작은 원통을 놓칠 수 있어 짧은 곡면의 법선을 처리하는 별도 비교 후보를 추가했지만 더 좋은 결과를 얻지 못했다. `plane_view_screen`과 `geometry_view_screen`에 총 320회의 비교와 동결 소스를 저장했다. 모두 2D 실험용 후보이며 ROS 기본 실행에 연결하지 않았다.

다른 학습 배치 40개(8500140–8500179)에서는 endpoint 29/40, edge 33/40, 대칭 후보 29/40이었다. 두 묶음 80개를 합치면 각각 60/80, 66/80, 64/80으로, 첫 40개의 87.5%가 그대로 유지되지 않았다. 추가 비교 120회의 접촉은 모두 0회였다. `symmetric_expansion`에 결과를 저장했다. 실행 도중 두 작업이 원인 불명의 종료 코드 143으로 중단되어, 동결 소스와 시나리오 해시를 검증하고 주행별 저장 방식으로 복구했다. 중단된 미완료 주행은 완료 실험 수에 포함하지 않는다.

첫 학습 묶음에서 가장 좋았던 후보의 가중치를 고정한 뒤 별도의 개발 검증 160개에 적용했다.

| 개발 검증 조건 | 기본 endpoint | 기존 edge | 대칭 후보 |
|---|---:|---:|---:|
| 전체 3개 발견 | 120/160 | 125/160 | 127/160 |
| 장애물 뒷면 | 29/40 | 30/40 | 28/40 |
| 장애물 부착 | 25/40 | 28/40 | 29/40 |
| 벽·장애물 혼합 | 28/40 | 29/40 | 31/40 |
| 벽면 부착 | 38/40 | 38/40 | 39/40 |
| 요청 장애물 32개 | 23/32 | 18/32 | 20/32 |
| 실패를 포함한 평균 시간 | 430.6초 | 420.3초 | 434.2초 |
| 접촉 | 0 | 0 | 0 |

대칭 후보는 기본 정책의 실패 18개를 성공으로 바꿨지만, 성공 11개를 실패로 바꿨다. 뒷면·고밀도 조건과 평균 시간이 나빠져 **기본값으로 채택하지 않았다**. 실패한 33개 임무에서 놓친 마커는 34개였고, 그중 29개는 한때 2m 이내를 지나갔다. 가까이 지나간 사실만으로 마커 앞면이 보였다고 해석하면 안 된다. `fidelity_validation/geometry_comparison.json`에 조건별 결과를 저장했다. 이 검증은 정책 개발용이며 예약한 독립 시험은 사용하지 않았다.

![공간 후보의 개발용 검증](training_robust/fidelity_validation/geometry_comparison.png)

대칭 후보 생성 계산도 최적화했다. 무작위 지도 120개에서 후보 순서까지 일치했고, 전체 주행 2개에서 모든 평가 값이 기존 실행과 정확히 같았다. 해당 합성 지도 묶음의 후보 생성 계산은 5.58초에서 0.79초로 줄었다. 이는 계산 시간이며 로봇 임무 시간 개선이 아니다. 근거는 `symmetric_sampler_equivalence.json`이다.

`symmetric_weight_search`에서는 학습 풀 400개에서 서로 다른 80개를 골랐다. 장애물 수와 부착면의 20개 유형별로 4개의 지도 방향을 사용했고, 40개씩 두 묶음으로 나눴다. 시작 가중치와 변형 5개를 첫 묶음에서 비교한 결과는 32/40, 32/40, 31/40, 27/40, 26/40, 32/40이었다. 전체·최저 부착면 성공률을 함께 고려해 고른 상위 2개를 두 번째 묶음에서 재평가하니 시작 가중치는 26/40, 변형 후보는 23/40이었고 endpoint 대조는 27/40이었다. 가중치 후보 평가 320회와 대조 40회, 총 360회다. **새 가중치를 채택하지 않았고 추가 개발 검증도 하지 않았다.** `best_policy.json`은 실험 내 최선인 기존 가중치의 보관본이며 배포용 개선 정책이 아니다.

별도로 20개 학습 배치에서 회전 모델을 60회 비교했다. 기존 일정 속도 모델은 18/20·평균 369.0초, 가속도를 포함한 0.7rad/s·1.6rad/s² 모델은 15/20·391.8초, Nav2 설정 상한인 1.0rad/s·3.2rad/s² 모델은 18/20·334.3초였다. 정책은 동일하다. 실제 달성 속도를 측정한 결과가 아니므로 기본 대리환경의 회전 모델이나 정책 성능 주장을 바꾸지 않았다. `angular_sensitivity`에 조건과 동결 소스를 저장했다.

실제 Gazebo 벤치에는 임무 시작부터 종료까지 처리된 카메라·odom·명령·clock의 주기, 회전/이동 속도, 실시간 계수와 `runtime_trace.csv` 기록을 추가했다. 측정이 없으면 `measured: false`와 null 값을 반환한다. 현재 환경에서는 실제 Gazebo 측정 자료가 없으며 통신 제한은 해결되지 않았다. 코드 테스트 **88개**와 ROS 패키지 빌드는 통과했다.

추가로 ROS 관측 메모리는 SLAM 지도의 크기나 원점이 바뀔 때 모든 관측을 지우고 있었다. `coverage_resize.py`는 해상도가 같고 원점 이동이 정수 격자이며 해당 셀과 3×3 주변의 점유/자유/미탐색 상태가 같은 경우에만 관측 기록을 옮긴다. 확장·축소에서 같은 물리 셀의 확정 및 부분 관측은 유지하고, 격자가 맞지 않거나 주변 형태가 바뀐 기록은 버린다. 벽의 반대쪽 관측 상태를 복사하지 않는다. 실제 주행 시간 개선을 측정한 것은 아니다. 가중치 학습은 이 변경 전 동결 코드로 진행했으며, 고정 크기의 대리환경 지도와 실제 SLAM 지도 확장은 구분한다.

이 수정까지 포함한 최종 전체 테스트는 **92개**, ROS 빌드도 통과했다. 고정 지도 주행 2개를 다시 실행하여 기존 결과와 모든 평가 값이 정확히 같음을 확인했다(`coverage_resize_replay.json`). 현재 실행 중인 실험은 없으며, 목표 성능을 달성한 것으로 처리하지 않았다.

## 추가 관측·방문 순서 실험

이번 단계에서는 서로 다른 **학습 배치 100개**에서 구조 비교 300회와 가중치 탐색·대조 평가 240회를 추가했다. 총 540회이며 아래의 개발용 검증 160개나 예약 시험 시드를 새로 평가하지 않았다. 기본 `surface_endpoint` 정책과 가중치는 유지했다.

같은 학습 배치 40개에서 기본 정책은 34/40, 면별 `edge`는 30/40, 관측 시간까지 계산하는 `view_greedy`는 30/40, 다음 두 지점의 관측 중복을 고려하는 `view_route`는 32/40이었다. 모두 접촉은 0회였다. `view_route_screen/completed.json`에 결과와 동결 소스를 보관한다.

이동 중 아직 보지 않은 면을 향해 멈춰 보는 `edge_glance`는 세 기준에서 27/40, 26/40, 28/40으로 나빠졌다. 평균 약 23/16/9회의 추가 관측 정지가 발생했다. 전체 관측 지점 집합을 고르고 이동 순서를 줄이는 `view_tour`도 학습 20개에서 10/20으로, 같은 배치의 기본 정책 17/20보다 낮았다. 두 방식 모두 채택하지 않았다. 이 모드들은 2D 실험용이며 ROS 기본 실행에 연결하지 않았다.

`view_route_search_v1`은 새로운 학습 20개에서 가중치 6개를 비교한 뒤, 상위 2개를 다른 40개에서 다시 평가했다. 가중치 후보 평가 200회와 기본 정책 대조 40회다. 최종 40개에서 시작 가중치는 28/40, 조정 후보 `[1, 1, 1, 1, 0.25]`는 29/40, 기본 endpoint는 33/40이었다. **이 후보도 채택하지 않았다.** `best_policy.json`은 해당 실험의 비교용 결과이며 배포 기본값이 아니다. 현재 모델의 정책 개선으로 발표할 결과는 아직 없다.

재현한 뒷면 마커 배치 2개에서는 회전이 전체 시간의 큰 부분을 차지했다. 요청 장애물 수 32개인 한 사례에서 endpoint는 이동 292.7초·회전 281.8초로 2개 발견 후 600초에 끝났고, view_route는 이동 291.8초·회전 186.7초로 518.0초에 3개를 찾았다. 다른 배치에서도 좋아진다는 뜻은 아니다. `view_time_diagnostic.json`은 5개 방식 × 2개 배치의 기록이며 원래 평가와 결과가 일치함을 확인했다.

정책 후보와 별개로 실행 오류 두 가지를 수정했다.

- ROS는 성공한 회전 관측에도 15초 제한을 걸어 가까운 다음 지점의 관측을 건너뛸 수 있었다. 성공 후에는 1.5초 카메라 대기를 두고 다음 관측을 허용한다. 실패한 회전의 30초 재시도 제한과 안전 복구는 유지한다. 0도 관측은 불필요한 Spin 액션 없이 대기한다.
- 대리환경은 1rad/s·5fps에서 각도 분할과 카메라 시간 조건이 충돌해 일부 프레임을 건너뛰었다. 회전 분할을 카메라 주기에 맞춰 수정했다. 기존 실험의 0.45–0.65rad/s·5fps 범위에는 이 분할 오류가 발생하지 않아, 위 결과가 이 수정으로 좋아진 것은 아니다.

현재 테스트 **77개**, 빌드 통과. Nav2 파일의 회전 속도 상한은 1.0rad/s, 가속도 상한은 3.2rad/s²이나 실제 달성 속도를 측정한 값은 아니다. 대리환경의 일정 회전 속도를 단순히 높여 나온 결과를 정책 개선이나 Gazebo 검증으로 주장하지 않는다. 실제 Gazebo 완주는 여전히 소켓 권한 문제로 미검증이다.

아래는 앞선 센서·격자 보정 단계의 기록이다.

## 5cm 격자에서의 개발용 검증

**최신 상태: 일반화 성능이 아직 부족하며 기본 정책은 유지했다.** 아래는 2D 대리환경의 개발용 검증이다. Gazebo 완주, 실제 로봇, 미사용 시험의 성공률로 해석하지 않는다.

현재 기본 월드는 추가 장애물 12개와 서로 다른 장애물의 뒷면에 붙인 마커 3개다. v2 학습 풀은 서로 다른 배치 400개, 검증은 새로운 배치 160개다. 장애물 8/12/16/24/32개 × 벽면/혼합/장애물/장애물 뒷면의 20개 유형을 균등하게 사용한다. 출발 자세, 지도 좌표 0/90/180/270도 회전, 마커 크기 0.12–0.30m, 인식 누락, 속도 제한도 바꾼다. 시드 8700000–8700199는 다음 독립 시험용으로 예약만 했으며 생성·평가하지 않았다.

이번에 확인한 문제와 수정:

- 대리환경의 기존 LiDAR는 지도에 고정된 120개 빔이었다. 설치된 TurtleBot4 모델의 640개 빔과 로봇에 따른 회전을 새 실험에 반영했다. 최고 점수가 나온 센서 설정을 고른 것이 아니다.
- 5cm/10cm 격자에서 경로를 단순화할 때 실제 로봇 위치 대신 격자 중심을 사용해 장애물 가까운 경로를 허용하던 문제를 수정했다. 관측된 지도만 사용하여 연속 경로 여유를 확인한다. 접촉 판정 반경은 바꾸지 않았다. 고정 재현 사례의 반복 안전 정지가 40회에서 1회로 줄었고, 1개 발견에서 561.3초 내 3개 발견으로 바뀌었다. 이는 단일 회귀 사례이지 일반 성공률이 아니다.
- `edge` 후보는 얇은 장애물의 각 면을 별도로 기록하고, 실제 카메라 관측 3회가 쌓인 면만 검사 완료로 처리한다. ROS 실행에도 비교 옵션으로 연결했다.
- LiDAR 계산을 C++로 가속했고 무작위·경계 자세 300개 및 전체 주행 4개의 결과 일치를 확인했다. 표면 방향 계산의 부분 갱신도 전체 재계산과 비교했으며, 전체 주행 2개에서 경로·지도·관측 횟수·평가 결과가 같고 실행 시간은 1.8–2.6배 빨랐다. 이 수치는 계산 시간 개선이며 로봇 주행 시간 개선이 아니다.

두 CEM 학습 `v2_endpoint`, `v2_edge`는 각각 **2세대·1,280회·서로 다른 배치 160개**까지 완료했다. 준비된 400개를 모두 학습한 것은 아니다. 두 실행 모두 검증에서 시작 가중치보다 좋은 가중치를 찾지 못했다. 이 실행의 동결 소스는 경로 수정 전 20cm 격자 모델이다. 현재 5cm 모델의 학습 결과로 간주하지 않는다. 이전 소스로 정확히 재개하려면 `recover_v2.py`를 사용하며, 새 학습 드라이버는 5cm 해상도와 연속 경로 검사를 설정에 명시한다.

경로 수정 후 SLAM 설정과 같은 **5cm 격자·640개 회전 빔·960×720·5fps·600초**로 동일한 v2 검증 160개를 평가했다. 후보 가중치는 평가 전에 고정했다.

| 검증 조건 | 기본 surface_endpoint | 실험 edge |
|---|---:|---:|
| 전체 3개 발견 | 120/160 (75.0%) | 125/160 (78.1%) |
| 장애물 뒷면 | 29/40 | 30/40 |
| 장애물 부착 | 25/40 | 28/40 |
| 벽·장애물 혼합 | 28/40 | 29/40 |
| 벽면 부착 | 38/40 | 38/40 |
| 장애물 32개 조건 | 23/32 | 18/32 |
| 600초 실패를 포함한 평균 시간 | 430.6초 | 420.3초 |
| 2D 접촉 | 0 | 0 |

edge는 기본 정책이 실패한 23개를 성공했으나, 기본 정책이 성공한 18개를 실패했다. 전체 개선은 5개에 그치고 가장 조밀한 조건은 악화되어 **기본 정책으로 승격하지 않았다**. 두 정책은 같은 배치를 평가했으므로 독립 배치 320개가 아니다. `fidelity_validation/comparison.json`과 각 `completed.json`에 조건별 결과를 보관한다. 이전 독립 시험의 65%와 이번 78.1%는 장면과 센서·격자가 달라 성능 상승으로 직접 비교하면 안 된다.

실패한 edge 주행 35개를 다시 실행하여 저장된 평가 결과와 완전히 일치함을 확인했다. 놓친 마커 38개 중 18개는 읽을 수 있는 앞면 위치에 도달하지 못했고, 16개는 앞면이 보이는 위치를 지나면서도 카메라를 그쪽으로 향하지 않았다. 나머지는 앞면 가림 2개, 픽셀 크기상 거리 부족 1개, 확정 프레임 부족 1개다. 이 분류는 실제 카메라 처리 시각과 대리환경의 가림·픽셀 조건으로 계산했다. 정답 기하는 진단에만 사용하며 정책에는 전달하지 않는다. 기록은 `fidelity_validation/failure_diagnostics/completed.json`과 각 주행의 NPZ에 있다. 다음 개선 대상은 관측 지점·방향과 방문 순서다.

![5cm 모델의 조건별 개발 검증](training_robust/fidelity_validation/comparison.png)

이 단계의 전체 테스트 **67개**, `colcon build`를 통과했다. `edge_ros_smoke.json`은 노드 생성과 설정·후보 생성의 확인이며 통신 성공 기록이 아니다. `gazebo_sensor_smoke/report.json`의 실제 시작 확인은 `getifaddrs`/UDP 소켓의 `Operation not permitted`로 실패했다. 이 환경에서 완료한 Gazebo 임무는 0개다. 아래의 준비된 Gazebo 벤치는 통신·렌더링이 가능한 호스트에서 실행해야 한다.

비교 실행:

```bash
source /home/peter/simul/install/setup.bash
ros2 launch osp_mission simulation.launch.py planner:=edge
```

기본 실행은 `planner:=surface_endpoint`다. 최신 상태는 `training_robust/latest_result.json`, `progress.json`에 기록한다. 다음 절부터는 이전 모델의 결과를 보존한 기록이다.

## 이전 독립 시험 — 120개 빔·20cm 모델

**최신 결론: 아직 충분한 일반화 성능을 달성하지 못했다.** 검증 배치에서 82.5%였던 관측 거리 2.4m·제한시간 관리 후보를 고정한 뒤, 미사용 장애물 배치 100개에서 기존 기본 정책과 비교했다. 결과에 따라 후보를 다시 조정하지 않았다. 새 후보는 사전에 정한 채택 조건을 통과하지 못해 **기존 기본 정책과 960×720·5fps 설정을 유지**했다.

| 미사용 100개 배치의 시험 조건 | 기존 기본 정책 | 새 후보 | 2D 접촉 |
|---|---:|---:|---:|
| 원래 지도 좌표 | 65/100 | 64/100 | 양쪽 0 |
| 같은 물리 배치, 지도 좌표 90° 회전 | 66/100 | 68/100 | 양쪽 0 |

두 좌표 조건은 같은 100개 기하를 반복한 것이며 독립 배치 200개가 아니다. 두 정책을 포함해 총 400회 평가했다. `training_robust/frozen_generalization_v1/registration.json`은 시험 장면을 읽기 전에 고정한 정책·소스·채택 기준을, `completed.json`은 결과를 보관한다. 시드 8300000–8300099는 이제 사용한 시험 세트이므로 다음 후보를 튜닝하는 데 재사용하지 않는다. **검증 78.75%/82.5%를 새 배치의 예상 성공률로 제시하면 안 된다.**

`diverse_directional_cem`과 `diverse_endpoint_v4`는 각각 **2,400회 학습을 완료**했다. 각 실행의 `coverage_audit.json`으로 첫 10세대에 서로 다른 200개 배치를 모두 사용했음을 확인했다. 장애물별 관측 확률을 바꾼 `endpoint_prior_elites`는 7세대·1,680회·140개 배치까지 저장했고 검증 77.5%였다. 이 실험은 전체 10세대 중 중간 체크포인트이며, 현재 실행 중인 프로세스는 없다. 단일 배치 학습은 아니다.

해상도 비교는 학습 배치 80개에서 별도로 수행했다. 960/1280/1600 폭의 이상적인 5fps 대리환경 성공률은 73.75%/76.25%/78.75%였으나, 잡음이 많은 합성 영상에서 1스레드 ArUco 디코딩 p95는 약 218/457/654ms였다. ROS·깊이 처리·Gazebo 렌더링은 포함하지 않은 CPU 측정이므로 실제 처리 속도나 인식률을 보장하지 않는다. 높은 해상도를 기본값에 적용하지 않았다. 자료는 `camera_probe`와 `decoder_speed.json`에 있다.

ROS 정책 파일에서 `settings.view_standoff`와 `settings.deadline_aware`를 검증·로딩할 수 있게 했다. 기존 파일은 2m·시간 관리 비활성으로 동작한다. `surface_wide_policy.json`은 독립 시험에서 탈락한 **비교용** 파일이다. 기한 직전 복구 회전 중 3개를 찾은 뒤 대기시간을 더해 성공을 실패로 바꾸던 대리환경 시간 처리도 수정했다. 단위 테스트 **54개**, 빌드 및 ROS 노드 설정 로딩을 확인했다. 통신·Gazebo 완주를 확인한 것은 아니다.

Gazebo용으로 4개 부착 유형 × 5개 장애물 수의 20개 배치와 정책 사본을 고정했다. ROS 통신과 렌더링을 사용할 수 있는 호스트에서 실행한다. 초기화 실패를 주행 실패와 별도로 기록하고, 초기화 실패가 반복되도록 계속 실행하지 않는다. 이 벤치는 아직 실행하지 않았다.

```bash
source /home/peter/simul/install/setup.bash
cd /home/peter/simul
python3 training_robust/gazebo_benchmark.py --run-prepared \
  --output training_robust/gazebo_balanced20
```

최신 상태는 `training_robust/latest_result.json`과 `progress.json`을 참고한다. 아래 검증 수치와 이전 실험 기록은 독립 시험 결과와 구분한다.

사용자의 추가 요청에 따라 기본 월드를 `config/robust_demo.yaml`로 변경했다. **추가 장애물 12개**, 마커 3개는 서로 다른 장애물의 시작점 반대 면에 부착한다. 기존 벽·기둥은 별도 고정 구조물이다. 4개 장애물 배치는 `assignment_demo.yaml`로 보존했다. 카메라는 960×720·5fps다. **시뮬레이션 기본 정책을 현재 2D 검증 최상위 후보인 `surface_endpoint`로 변경**했고 가중치는 `config/surface_endpoint_policy.json`에 고정했다. Gazebo 완주 검증은 아직 없으며 실제 로봇용 launch 기본값은 legacy다.

`scenarios_robust`는 기존 학습 100개에 새로운 배치 100개를 더한 **학습 200 / 검증 80 / 시험 100개**다. 장애물 요청 개수 8/12/16/24/32와 벽면 3개, 벽·장애물 혼합, 장애물 3개, 장애물 뒤쪽 3개를 조합한다. 실제 장애물 개수를 각 YAML에 저장한다. 시작 위치·방향, 0.12–0.30m 마커 크기, 8%/20% 인식 누락, 0.20/0.23/0.26m/s 속도 제한을 바꾼다. 정답 위치는 평가에만 사용한다. 접근 가능한 배치만 생성한다. 이러한 제한된 범위에서의 검증도 모든 가능한 문제의 성공 보장은 아니다.

`train_diverse/manifest.json`에 200개 시드와 20개 유형별 개수를 기록했다. 장애물 기하 해시가 모두 다르고 검증 세트와 겹치지 않는지 확인했다. 매 세대 4개 부착 유형 × 5개 장애물 수에서 하나씩, 총 20개 배치를 선택한다. 유형별로 모든 배치를 사용한 뒤 다시 섞으므로 **첫 10세대에 200개 배치를 빠짐없이 사용**한다. `current_batch.json`과 `history.json`에 실제 사용한 시드를 남긴다. 기존 실행은 중복 허용 추출을 사용했으며, 실행 중인 프로세스가 새 방식으로 바뀌었다고 간주하지 않는다.

원인 분석으로 가까운 표면의 시선 검사 오류, 얇은 칸막이의 표면 방향 누락, 한쪽 면을 보고 반대쪽까지 검사 완료로 처리하던 문제를 수정했다. 현재 `directional` 정책은 관측 지도에서 방향을 추정하고 픽셀 크기·시야·입사각·독립된 3회 관측을 사용한다. 카메라 시계도 실제 설정에 맞게 5fps로 모델링했다. 이전의 드문 관측 타이밍으로 측정한 수치와 직접 비교하면 안 된다.

학습은 충돌을 우선 줄이고 전체 및 가장 취약한 부착 유형의 성공률을 높이는 CEM 가중치 최적화다. **`diverse_directional_cem`과 `diverse_endpoint_v4`가 각각 10세대 × 20개 배치 × 12개 후보 = 2,400회**를 완료했으며 별도 검증도 추가로 실행했다. 이전 `endpoint_cem`과 `directional_cem`은 각각 1,440회를 완료했다. 실행별 `source_snapshot`/`configuration.json`이 적용 소스와 해시를 보존한다. 예전 스냅샷의 동명 train.py 충돌은 `snapshot_repair.json`에 설명하고 패키지 모듈을 별도 보존했다. 새 실행부터 드라이버는 `training_driver.py`로 저장한다. `completed.json`이 있어야 해당 유한 학습이 종료된 것이다. 실시간 상태는 `training_robust/progress.json`과 각 실행의 `history.json`을 확인한다.

`diverse_directional_cem`의 첫 프로세스는 1,200회 완료 후 종료 코드 143으로 끝났다. 원인은 확인되지 않았다. 저장한 원래 시뮬레이터·정책 소스, CEM 평균·분산·가중치, 난수 상태, 미사용 배치 순서를 복원해 이어서 실행했다. `recoveries.json`에 검증 기록이 있다. 현재 코드를 원래 소스인 것처럼 사용하지 않았다. 새 학습은 `--generation-limit 4`로 한 프로세스의 실행량을 제한하며, `chunk_finished_*.json`은 전체 학습 완료를 뜻하지 않는다. 다음 프로세스에서 동일 설정에 `--resume`을 더하면 이어진다. 원래 코드와 다르면 복원을 거부한다.

검증 80개·5fps 조건에서 기존 legacy는 61.25%였다. 완료한 endpoint 학습은 78.75%지만 접촉 1회가 남았다. directional의 새 학습 시작 후보는 76.25%·접촉 0회, 성공 실행 평균 388.4초다. 모두 2D 대리환경의 정책 선택용 검증 수치이며 미사용 시험 또는 Gazebo 결과가 아니다. endpoint 실행은 광학 깊이 8m 상한 추가 전 소스를 사용했으므로 최종 비교 시 현재 공통 소스로 재평가해야 한다.

이후 접촉 진단에서 연속 위치의 정지 검사와 격자 중심의 접촉 검사 기준이 달랐음을 확인했다. 접촉 반경 0.22m·판정은 그대로 두고, 이미 관측한 장애물에 대해 같은 격자 기준의 사전 정지를 추가했다. `timing_v3`의 과도한 격자 팽창은 성능을 낮춰 제외했고, `timing_v4`는 원래 연속 위치 안전 여유를 유지하면서 접촉 판정에 해당하는 경우만 추가 정지한다. 이미 실행 중인 학습은 시작 시점 소스를 계속 사용하므로 최종 후보는 현재 모델에서 재검증해야 한다.

`timing_v4`의 동일한 소스·80개 배치·카메라 5fps 비교 결과:

| 정책 | 3개 완수 | 2D 접촉 | 성공 실행 평균 |
|---|---:|---:|---:|
| legacy | 49/80 (61.25%) | 0 | 398.7초 |
| surface_endpoint 학습 후보 | 63/80 (78.75%) | 0 | 363.6초 |
| directional 학습 시작 후보 | 61/80 (76.25%) | 0 | 388.4초 |

성공 집합이 다르므로 성공 실행 평균만으로 속도 향상을 단정하지 않는다. 위 표는 검증 세트의 수치다. 이후 수행한 독립 시험은 문서 맨 위에 기록했다. 두 후보 모두 ROS 실행 코드에 연결했지만, 위 수치가 Gazebo에서 재현됐다는 뜻은 아니다.

기본 시연 배치의 공통 2D 조건 비교는 legacy **600초에 2/3**, surface_endpoint **340.6초에 3/3**, directional **574.5초에 3/3**이며 접촉은 모두 0회였다. `current_demo_comparison.json`과 `default_candidate_route.png`에 저장했다. 한 시연 배치를 전체 성능으로 대체하지 않는다.

`surface_endpoint`와 `directional`은 실제로 처리된 RGB 프레임의 CameraInfo와 0.25초 이내 LiDAR, 해당 시각 TF가 있어야 관측을 누적한다. 같은 프레임의 중복이나 카메라에서 가려진 표면은 완료 처리하지 않는다. directional은 방향별 8개 관측 상태를 유지하고, endpoint는 서명 있는 표면 법선과 셀별 관측 상태를 사용한다. SLAM 지도 크기·원점이 바뀌면 관측을 초기화한다. 카메라가 끊기면 목표를 취소하고 대기한다. **Gazebo 완주 검증은 실행기의 네트워크 인터페이스 권한 오류와 렌더링 장치 부재로 남아 있다.**

추가 실험에서 면을 정면으로 보는 후보를 늘린 `face_candidates`는 검증 67.5%, 정지 관측·회전 각도를 선택하는 `gaze`는 검증 53.75%로 낮아 채택하지 않았다. gaze는 학습 배치 20개에서 85%였으나 검증 80개의 뒷면 유형에서는 20%였다. 작은 학습 표본의 개선을 최종 성능으로 사용하지 않는다. 분석 자료는 `face_diagnostics.json`, `failed_routes.png`, `scan_time_diagnostics.json`, `face_ablation_v1`, `gaze_ablation_v1`이다. 이들 추가 모드는 ROS에 연결하지 않았다.

```bash
# 현재 최상위 2D 후보가 기본값. Gazebo 완주 성능은 미검증.
ros2 launch osp_mission simulation.launch.py
# 비교 모드
ros2 launch osp_mission simulation.launch.py planner:=directional
ros2 launch osp_mission simulation.launch.py planner:=legacy

# 새 출력 폴더를 지정해야 기존 학습 이력을 덮어쓰지 않는다.
PYTHONPATH=src/osp_mission OPENBLAS_NUM_THREADS=1 python3 training_robust/train.py \
  --output training_robust/new_diverse_run --planner directional --camera-hz 5 \
  --train-dir scenarios_robust/train_diverse --generations 10 \
  --initial-policy training_robust/diverse_initial.json
```

- 배치 미리보기: `training_robust/robust_preview.png`
- 다양한 배치 20개 미리보기: `training_robust/diverse_layouts.png` (정답 표시가 있는 오프라인 그림, 정책 입력 아님)
- 5fps 검증·실패 목록: `training_robust/timing_v2/legacy_validation80.json`
- 배치 다양성 증빙: `scenarios_robust/train_diverse/manifest.json`, `training_robust/diversity_audit.json`
- 재생성: `PYTHONPATH=src/osp_mission python3 -m osp_mission.robust_scenarios`
- 강화 기본 배치 실행: 기존 `ros2 launch osp_mission simulation.launch.py` 사용
- 최신 학습 수치는 `training_robust/diverse_directional_cem/history.json` 확인. 첫 세대 완료 전에는 `current_batch.json`을 확인한다.
- 검증: `training_robust/endpoint_ros_tests.log`의 단위 테스트 40개 통과, `endpoint_ros_build.log` 빌드 성공. `endpoint_ros_smoke.json`은 노드 생성·가중치 로딩만 확인하며 transport 통신 성공을 의미하지 않는다.

아래 내용은 이전 4개 장애물 프로필의 실험 기록이다. 그 성공률을 강화 배치의 성공률로 사용하면 안 된다.

---

# PDF 과제용 시뮬레이션 — 2026-11-05 현재 기본값

사용자가 제공한 OSP_final_project.pdf의 2–5, 8쪽에 맞춰 별도 과제 프로필을 만들었다. 아래 기존 숨김 목표물 기록과 구분한다.

## 배치와 실행

도면의 20.15×11.35m 공간 근사 모델을 유지하고 PDF 예시의 박스 4개를 참고했다. 박스 크기(약 0.65–0.95×1.1–1.6m, 높이 1m), 위치와 마커 크기(0.16–0.30m)는 **시뮬레이션 설계 가정**이며 실제 시연 조건을 알고 정한 값이 아니다. 중앙 빗금·차선은 바닥 표시이며 충돌 장애물이 아니다. 실제 벽 돌출부와 트랙 곡선은 도면 이미지 기반 근사다.

기본 `assignment_demo.yaml`은 벽 2개·박스 1개의 수직면에 마커를 붙인다. 부착 구조물은 바닥까지 이어진다. 알고리즘에는 정답 위치·ID·완성 지도를 제공하지 않는다. 시나리오별 시작 자세는 고정하되 평가에서는 여러 모서리와 방향으로 바꾼다. ID와 크기도 변경한다. 실제 시연에서 가능한 모든 부착 조합을 검증한 것은 아니다.

```bash
cd /home/peter/simul
source install/setup.bash
export ROS_DOMAIN_ID=48
ros2 launch osp_mission simulation.launch.py
```

센서와 Nav2가 준비된 뒤 별도 터미널에서:

```bash
source /home/peter/simul/install/setup.bash
export ROS_DOMAIN_ID=48
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

기본값은 `scenario:=.../config/assignment_demo.yaml`, `planner:=legacy`, `camera_width:=960`, `policy_file:=.../config/assignment_policy.json`, `nav2_params:=.../config/nav2_assignment.yaml`이다. 자동 시작하지 않으며 기존 시작 신호 절차를 유지한다. 마커 3개 발견 또는 600초에 종료하고 기존 map/marker 결과를 저장한다. 기본 정책은 기존 CEM 가중치를 보존했다. 새 inspection 정책의 648회 학습은 기존 정책을 대체하지 못했다.

`planner:=inspection`은 관측 표면 검사 후보를 사용하는 비교용 기능이다. policy_file을 생략하면 해당 모드용 파일을 자동 선택한다. `robot.launch.py`의 실제 로봇 기본값은 기존 legacy와 기본 Nav2 설정을 유지하며, Gazebo 카메라 해상도 설정은 실제 OAK-D를 변경하지 않는다.

## 수정한 문제

- 20cm 격자 셀의 중심만 검사해 서쪽·남쪽 12cm 벽이 누락되던 오류를 수정했다. 모든 경계 벽과 겹치는 셀을 점유 처리한다. **이전 학습 환경과 수치를 직접 비교하면 안 된다.**
- 실제 Gazebo 카메라 시야각 1.25rad로부터 초점거리를 계산하도록 대리환경을 수정했다. 기존 172px 가정 대신 폭에 맞는 값을 사용한다.
- RGBD 센서를 960×720·5fps로 구성했다. RGB와 depth는 동일 센서이며 CameraInfo를 그대로 사용한다. 320×240, 640×480 비교도 수행했다. 설치된 ROS 모델 파일은 수정하지 않았다.
- 비스듬한 마커의 픽셀 축소와 0.15초 이상 간격의 3회 확인을 2D 인식 모델에 반영했다. RGB 영상 디코딩 자체를 재현하는 것은 아니다.
- MPPI 속도 상한을 smoother와 같은 0.26m/s, 회전 상한 1rad/s로 맞춘 별도 Nav2 설정을 적용했다. MPPI 시각화는 끄고 충돌 회피·후퇴 안전 검사는 유지했다.

## 최종 독립 평가

학습 80개, 검증 24개, 비교 48개로 구성을 선택했다. 비교 세트에서 legacy+960이 inspection+960보다 높아 전자를 고정했고, **새 시드 740000–740047의 48개**에서 최종 평가했다. 각 난이도는 16개이며 장애물 수 4/6/8, 목표 3개, 제한 600초, 최대 속도 0.26m/s다. 최종 평가 후 가중치를 다시 조정하지 않았다.

| 지표 | 기존 정책+320×240 | 최종 정책+960×720 |
|---|---:|---:|
| 3개 모두 발견 | 4/48 (8.3%) | 37/48 (77.1%) |
| 평균 발견 수 | 1.208 | 2.792 |
| 전체 평균 시간 | 590.3초 | 420.4초 |
| 성공 실행 평균 시간 | 483.8초 | 367.0초 |
| 2D 접촉 | 0 | 0 |

최종 성공률: 장애물 4개 75.0%, 6개 68.8%, 8개 87.5%. 난이도별 표본이 작고 마커 크기·방향·출발 위치도 달라 장애물 수와 성공률이 단조 관계는 아니다. 성공 실행 평균은 양쪽의 성공 배치 집합이 다르므로 단독 속도 비교로 쓰면 안 된다.

기본 배치의 최종 정책 결과는 **410.7초, 3개 발견, 2D 접촉 0회**, 경로 길이 55.5m다. 2m 이내 접근은 2개만 확인되어 PDF 배점상 만점을 달성했다고 주장하지 않는다. 실제 RGB 인식, 지도 좌표 오차, 접근 점수, 물리 접촉과 960px 렌더링 처리량은 Gazebo에서 별도 검증해야 한다.

- 배치·경로 그림: `training_assignment/assignment_preview.png` (오프라인 배치 검토용 정답 표시; 정책 입력 아님)
- 수치와 소스 해시: `training_assignment/final_report.json`
- 최종 개별 결과: `final_selected.json`, `final_baseline.json`
- 재현: `PYTHONPATH=src/osp_mission OPENBLAS_NUM_THREADS=1 python3 training_assignment/benchmark.py --split final_test --planner legacy --camera-width 960 --output /tmp/assignment_eval.json`
- 배치 재생성: `ros2 run osp_mission generate_assignment_scenarios --output scenarios_assignment`

빌드, 테스트 19개, 200개 배치의 접근 가능성·재현성 검사, 생성한 실제 로봇 URDF의 RGBD 960×720·5fps 설정을 확인했다. 현 제한된 실행기에서는 ROS/Gazebo 브리지가 네트워크 인터페이스 권한 오류로 종료하므로 **Gazebo 완주 검증은 아직 수행하지 못했다**. 이전 숨김 과제의 검증 완료 정책으로 표시하거나 실물 성능으로 해석하지 않는다.

---

# 이전 숨김 과제 실험 기록

# 장애물 탐색 정책 학습과 실시간 화면

## 실행 중인 구성

- 상자·원통·선반·가림막을 섞은 현재 숨김 목표물 배치 200개: 학습 120 / 검증 32 / 최종 시험 48.
- 난이도는 장애물 요청 개수 8/16/24/32로 나눈다. 배치 공간이 부족하면 일부 개체를 생략하므로 실제 개수는 YAML의 obstacles 길이로 확인한다. 초기 200개 배치의 실제 개수는 8–32개다.
- 모든 마커는 Original ArUco이며, ID·크기·장애물 부착면을 변경한다. 세 마커를 서로 다른 장애물의 시작점 반대 면에 부착한다. 시작점에서는 보이지 않으며 로봇이 이동해 찾아야 한다. 벽에 붙이지 않는다. 로봇 크기를 반영한 접근 가능성과 마커 앞 관측 위치를 검사한다.
- Cross-Entropy Method(CEM)로 탐색 후보의 가중치와 회전 관측량을 학습한다. 바퀴 속도를 직접 학습하는 신경망 정책이 아니다.
- 로봇 실행 시 학습 정책은 지도·관측 이력에서 다음 목표를 고른다. 이동과 충돌 회피는 Nav2 및 Collision Monitor가 수행한다.

## 화면

왼쪽 위 Gazebo: 실제 3D 환경, 로봇과 장애물.
왼쪽 아래 RViz: 실제 LaserScan / SLAM 지도 / 마커 표시.
오른쪽 OSP Training Live: 세대별 지표와 2D 센서 재생.

오른쪽 재생은 현재 체크포인트를 별도 2D 에피소드에서 재생하는 것이다. 학습 워커 화면이나 Gazebo 물리 시뮬레이션이 아니다. 경로·점유 지도는 해당 재생 환경의 센서 관측으로 만들어진다. 대시보드가 닫혀도 학습은 계속된다.

```bash
cd /home/peter/simul
source install/setup.bash
ros2 run osp_mission training_dashboard --run training_hidden/initial --scenarios scenarios_hidden
```

## 학습 방식과 성능 해석

빠른 2D 환경에서 600초 에피소드를 반복한다. 부분 관측 LiDAR 지도, 제한된 카메라 시야·마커 픽셀 크기에 따른 인식 거리·가림·부착면 방향·관측 누락·주행 속도 변화를 모델링한다. 실제 SLAM 오차, RGB 영상 디코딩, 모터 동역학과 동적 장애물은 재현하지 않는다. 따라서 2D 결과를 Gazebo나 실물의 성공률로 사용할 수 없다.

정책 입력은 관측된 지도, 이동 비용, 회전 비용, 방문 이력, 여유 거리, 미관측 표면이다. 목표물 정답 위치·ID·전체 장애물 지도는 입력하지 않는다. 정답은 환경의 센서 합성과 보상/평가에만 사용한다.

보상: 발견 개수, 세 마커 완료 보너스, 시간·접촉 패널티와 지도 관측량. 체크포인트는 검증 배치의 성공률, 발견 개수, 소요 시간 순으로 선택한다. 최종 시험 배치는 CEM 최적화에 전달하지 않는다.

이전 벽면 타깃 과제의 v1(폐기): 7,680개 학습 에피소드. 검증 최고 28.1%, 최종 시험 48개에서 baseline/learned 모두 14.6%, 접촉 0회. 일반화 개선이 입증되지 않아 현재의 장애물 숨김 과제와 다르므로 현재 조건의 성능으로 사용하지 않는다. 기존 기록은 training_runs/에 보존했고 해당 학습은 중단했다.

이후 라운드는 물체 내부의 점유 셀을 미관측 표면으로 잘못 세던 보상 특징을 수정했다. 관측 가능한 경계 표면과 가까운 카메라 관측 영역만 계산한다. 이 변경 후의 지표는 v1과 별도로 기록한다.

## 자동 반복과 검증 기준

현재 `training_hidden`의 `continual_train` 감독 프로세스가 숨김 목표물 초기 라운드 뒤에도 다음 라운드를 자동 실행한다. 라운드마다 학습용 120개·시험용 48개에 새 시드를 사용하며, 비교를 위한 검증용 32개는 고정한다. 라운드의 검증 최고 정책으로 다음 라운드를 시작한다.

- 2D의 새 시험 48개에서 성공률 90% 이상, 접촉 0회여야 Gazebo 검증으로 진행한다.
- 고정한 후보를 Gazebo의 실제 LiDAR·SLAM·RGB ArUco 검출로 시험한다. 8개 배치 모두에서 600초 내 세 마커 발견, 접촉 0회, 결과 저장 성공이 필요하다.
- 두 기준을 통과하면 `verified_policy.json`을 생성하고 학습을 종료한다. 이는 유한한 시험에서 검증한 정책이지 전역 최적성의 증명이 아니다. 실물 검증은 별도다.
- 기준 미달이면 새 배치로 계속 학습한다. 현재는 최적 정책이 완성된 상태가 아니다.

```bash
# 이미 실행 중이면 중복 시작하지 않는다. flock으로 중복 감독 실행을 차단한다.
ros2 run osp_mission continual_train --root training_hidden --initial training_hidden/initial --workers 4

# 사용자가 원할 경우, 현재 라운드 종료 후 반복을 중단하는 표시 파일
# touch training_hidden/STOP_AFTER_ROUND
```

프로세스 ID는 `training_hidden/continual.lock`에 기록된다. 진행 상태는 `training_hidden/live_state.json`, 라운드별 학습 그래프는 `history.json`, 정책은 `best_policy.json`, 시험 결과는 `test_report.json` / `test_episodes.csv`에 남는다. 소스 사본과 해시도 라운드별로 보관한다.

`training_hidden/deployed_policy.json`은 검증 최고 정책이며 아직 Gazebo 검증 완료를 뜻하지 않는다. 실제 로봇에는 `verified_policy.json`의 존재 및 실물 시험 결과를 확인한 뒤 적용한다.

## 재현

```bash
ros2 run osp_mission generate_scenarios --output scenarios_hidden --train 120 --validation 32 --test 48
ros2 run osp_mission train_policy --scenarios scenarios_hidden --output training_hidden/manual \
  --generations 24 --population 20 --batch 16 --workers 4
ros2 launch osp_mission simulation.launch.py \
  scenario:=/home/peter/simul/scenarios_hidden/validation/10003.yaml \
  policy_file:=/home/peter/simul/training_hidden/deployed_policy.json
```

단일 Gazebo 평가:

```bash
ros2 run osp_mission gazebo_eval \
  --scenario scenarios_hidden/test/20000.yaml \
  --policy training_hidden/deployed_policy.json \
  --output training_hidden/gazebo_trial
```

시뮬레이션 시간 기준 600초, 프로세스 시작·센서 초기화 실패와 벽시계 감시 제한은 별도 실패로 기록한다. 실제 bump hazard를 2초 간격의 접촉 사건으로 집계한다. 2D의 접촉 횟수는 래스터 충돌 모델의 결과이며 이 물리 센서 집계와 구분한다.

## 검증

```bash
colcon build --symlink-install
colcon test --packages-select osp_mission
colcon test-result --verbose
```

배치/마커 재현성, 접근 가능성, SDF 충돌 형상, 정책 파일 검사, 관측된 영역만을 통한 경로 선택, 벽 뒤 타깃 비공개, 충돌 좌표 반올림 회귀 등을 검사한다.

알고리즘과 안전 계층 참고: [CEM 정책 탐색 원 논문](https://cdn.aaai.org/ICML/2003/ICML03-068.pdf), [Nav2 Jazzy Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/).

SLAM의 미관측 구멍을 실제 점유 장애물처럼 팽창시켜 후보를 0개로 만들던 주행 문제를 수정했다. 경로는 관측된 자유 셀에서만 연결하고, 목표점 주변의 관측 밀도를 검사한다. 장애물 여유 거리는 실제 점유 셀에서 계산하며, 로봇 footprint와 최종 이동 가능 여부는 Nav2가 검사한다.

## 낮은 성공률 원인 조사

첫 Gazebo 600초 실행은 마커 1개, bump 사건 7회로 종료했다. 로그에는 `Collision Ahead - Exiting Spin` 203회가 있어, 막힌 위치에서 회전을 반복하는 복구 문제를 확인했다. 실패 회전은 30초 동안 재시도를 억제하고, 후방 scan이 0.55m 이상 비어 있을 때만 Nav2 BackUp으로 0.20m 후퇴한 뒤 재계획하게 수정했다. 후퇴에도 Nav2 충돌 검사를 적용한다. 이 변경으로 최종 성공률이 개선되었는지는 새 Gazebo 시험에서 확인해야 한다.

## 2026-11-04 추가 진단·학습 결과 — 개선 미입증

`training_hidden/tuning_20261104/experiment_report.json`에 전체 비교를 보관했다. 기존 체크포인트를 동결하고 4개 CEM 학습군과 2개 무작위 가중치 탐색에서 총 **9,088개 학습 에피소드**를 실행했다. 600초, 숨김 마커 3개, 장애물 요청 개수 8/16/24/32 조건은 완화하지 않았다. 인식 거리와 표면 검사 거리의 일치, 후보점의 여유 거리, 긴 벽의 가중치, 가시선 기반 표면 점수, 목표 도착 방향, 회전량, 난이도 균등 배치 및 성공률 우선 목적함수를 비교했다. 모든 수정이 개선으로 이어진 것은 아니다.

검증 32개에서 선택한 재학습 후보는 기존과 성공률 37.5%로 같고 평균 발견 수가 1.969→2.031이었다. 이후 후보를 고정하고 새로운 시드 910000–910047의 48개 시험에서 평가했다.

| 지표 | 기존 정책 | 재학습 후보 |
|---|---:|---:|
| 세 마커 성공 | 14/48 (29.2%) | 11/48 (22.9%) |
| 평균 발견 수 | 1.875 | 1.771 |
| 평균 시간 | 546.2초 | 566.9초 |
| 2D 접촉 | 0 | 0 |

**후보는 채택하지 않았고 기존 deployed_policy.json을 변경하지 않았다.** `selected_policy.json`은 검증 세트에서 선택되었으나 새 시험에서 탈락한 실험 산출물이다. `verified_policy.json`을 생성하지 않았다. 최적 설정이나 성공률 90% 달성을 주장할 근거가 없다. 2D 결과를 Gazebo 성공률로 해석하면 안 된다.

진단 근거와 제한:

- 기존 정책의 검증 8개 계측에서 평균 558.5초 중 관측 회전에 162.4초, 이동에 266.2초를 사용했다. 회전량 단독 축소는 이번 비교에서 효과가 없었다.
- 작은 마커의 대리환경 인식 거리는 약 1.45m인데 표면 검사 이력은 3.5m까지 누적한다. 이 불일치를 수정한 비교군도 새 가중치 학습 후 기존 검증 성능을 넘지 못했다. 따라서 해당 변경은 기본 코드에 적용하지 않았다.
- 같은 검증 32개를 반복 사용한 상태에서 발견 수의 작은 향상이 새 배치에 일반화되지 않았다. 새 시험 결과로 후보를 다시 고르거나 재학습하지 않았다.
- Nav2 MPPI의 선속도 상한은 0.5m/s, smoother 출력 상한은 0.26m/s다. 대리환경 속도도 0.22–0.31m/s여서 실제 설정과 차이가 있다.
- 실제 Gazebo는 로그 경로를 쓰기 가능한 위치로 옮긴 뒤에도 `getifaddrs: Operation not permitted` 및 브리지 종료로 시작에 실패했다. 현재 실행기에 `/dev/dri`도 없다. 3D 성능은 검증되지 않았다.

기본 Nav2 설정은 유지했다. 속도 상한을 smoother와 맞추고 MPPI 시각화를 끈 **미검증 비교 후보**는 `training_hidden/tuning_20261104/nav2_candidate.yaml`이다. `simulation.launch.py`와 `robot.launch.py`에 `nav2_params` 인자를 추가해 기본 설정을 덮어쓰지 않고 비교할 수 있다.

ROS 네트워크와 EGL이 허용된 호스트에서 후보 설정을 별도로 검증하려면:

```bash
cd /home/peter/simul
source install/setup.bash
ros2 launch osp_mission simulation.launch.py \
  policy_file:=/home/peter/simul/training_hidden/tuning_20261104/prior_policy.json \
  nav2_params:=/home/peter/simul/training_hidden/tuning_20261104/nav2_candidate.yaml
```

배치 평가용 `gazebo_benchmark.py`도 보관했다. 이 스크립트는 **탈락한 재학습 후보**를 진단용으로 실행하므로 운영 정책으로 사용하지 않는다. 평가기는 ROS 초기화 실패를 기록하며 초기화 전에 시뮬레이터를 시작하지 않는다. launch 인자 확인 및 pytest **14개 통과**. 이번에 시작한 유한 학습 실험은 모두 종료했다.


## Native update: 2026-11-05 10:50 KST

- Live communication rechecked: workspace writes, TCP/UDP/multicast loopback and actual domain99 camera, scan, odometry, IMU, clock and mission status all succeeded. Evidence: `training_robust/native_diagnostics/permission_recheck_20261105_0146.json`. No extra user permissions are needed for these local tests.
- Panorama v5 completed the 32-obstacle hidden-marker development layout in **293.6 simulation seconds**, all3 correct IDs, zero contacts, maximum coordinate error **0.196m**. Video (1470 frames, 5fps) and saved maps passed artifact auditing. This candidate failed the 16-obstacle layout, so its current development result is **1/2**, not a90% policy. Source122 files remained unchanged.
- Its full video is `training_robust/native90/panorama_v5_screen2/episodes/8600139/video/mission.mp4`.
- Persistent v4 evaluator exited143 while its simulator continued. The cause is unestablished; cgroup OOM counters were zero. Saved the partial mission/maps/video (258s), explicitly recorded `evaluator_interrupted`, and preserved the two earlier completed failures. The partial run is excluded from policy comparisons. An identical frozen-source repeat is now running as `persistent_map_v4_recovery32` on domain96.
- Marker focus v6 started automatically after panorama finished. Domain100 is running with active Nav2 and drive controllers. Source124 files verified unchanged, production18-pixel/3-frame confirmation retained. It will test16/24/32 obstacles with recorded video.
- The partial persistent32 shadow decoder confirmed only ID382, also confirmed by production. The panorama32 shadow ended with ExternalShutdownException and produced no usable result. Neither justifies lowering production detection thresholds.
- Reserved40 layouts remain unused; >=36 complete valid successes are still required for the final90% target.


## User-required approach revision: 2026-11-05

The user explicitly requires every marker to be approached within **1.5m**. Previous `len(targets)>=3` termination did not enforce this and is superseded. Mission completion now requires three distinct accepted close RGB observations for each of the three confirmed IDs. Image-time map-to-base TF measures the robot position; the operating radius is1.3m, with0.9/1.05/1.2m free-space goals to allow margin. No target ground truth enters navigation. The evaluator checks actual simulator trajectory distances<=1.5m independently, and the final40-scene gate requires this protocol.

Main157 tests and candidate177 tests passed; both builds passed. Two frozen service campaigns `approach_v8_screen3` and `approach_rpp_v8_screen3` compare MPPI/RPP on16/24/32 obstacles with real video. Their code differs only in Nav2 controller settings. The old service campaigns were explicitly stopped after saving partial maps/videos because the user changed the completion criterion; these were intentional stops, not unexplained failures.

Historical re-evaluation is `training_robust/native_diagnostics/reassessment_approach15.json`: panorama32(293.6s) fails full approach because ID382 remained3.10m away; the earlier motion16 result approached all IDs at0.448/0.600/0.879m. Historical source evaluations are preserved. No90% claim is warranted.


## Verified status and result-file race: 2026-11-05

Under1.5m criteria, RPP completed3development layouts:16passed386.903s,24timed out despite3correctIDs/all3actual approaches because ID578 lacked3close RGB frames,32timed out with ID776 missing. All had0contacts. Automated completion1/3 is not a generalization estimate. MPPI16passed512.519s. MPPI24saved3close-confirmed IDs in169.2s, but the evaluator read result.json during its write and crashed with JSONDecodeError, so its final coordinate audit is missing. The exact traceback is preserved in approach_v8_screen3/service.log. These services did not exhibit the earlier unexplained143 termination.

Fixed result publication with temporary-file atomic replacement and made the evaluator retry incomplete JSON within its existing watchdog.179candidate tests passed and build passed. A frozen copy runs24/32 again as approach_v9_io_remaining2 under osp-approach-v9-io.service. Policy,1.5m requirement, and near confirmation are unchanged. Historical files are preserved.


## Native parameter search launched: 2026-11-05

`native90/native_search_v1` is a systemd-supervised actual Gazebo search. Four frozen candidates compare MPPI baseline, increased frontier weight, increased travel-cost weight, and RPP baseline. Phase1 uses5shared new layouts spanning8/12/16/24/32 obstacles and wall/mixed/obstacle/backside mounting (20native episodes). Phase2 evaluates the top2 on20fresh balanced layouts (40episodes). Only if the winner succeeds>=18/20 is it frozen and evaluated on the untouched40reserved layouts. Thus60trials are planned, up to100if the holdout gate is reached. The final90% claim still requires the independent36/40gate.

The coordinator is `osp-native-search-v1.service`; workers each have6GiB memory limits and actual videos. At most2native benchmark processes run, including the pre-existing24/32 recovery job. It stops issuing new runs for3consecutive infrastructure failures, <8GiBfree disk, or frozen-source/support-file changes. Policy runtime uses only observed maps, sensors and decoded IDs. Evaluator target coordinates are confined to simulation construction and after-run scoring. All previous results remain preserved.

Driver ranking and manifest tests pass. The RPP IO-fix stage differs from the179-test MPPI stage only in the already-built controller configuration. Once the batch is running, do not modify its stages or hash-locked benchmark, audit, service-launch and coordinator scripts.

Native tournament startup check (2026-11-05 19:28 KST): trial000 terminated as startup_failure after180 wall seconds. Sensors and Nav2 were active, but evaluator had no mission status/successful start; root cause remains unproven. Trial001 on the same source and domain110 entered active navigation, found its first marker, and had zero contacts at58.6 sim seconds. Coordinator was briefly stopped for diagnosis and resumed without changing frozen sources; original failure remains in the denominator. Authoritative changing status: native90/native_search_v1/state.json.

## Sequential failure fixes, 2026-11-06

Implemented immutable v10 escape, v11 independent 1.5m approach, v12 conflicting-ID resolution, v13 observed-face/revisit candidates. Final candidate source: `training_robust/native90/runtime_fix_v13_observation_staging/src/osp_mission`; 189 tests passed and build passed. Previous main/default policy is not automatically replaced by this unvalidated candidate. Three originally empty snapshots now have128/113/161 candidates;4/5 completion failure traces finish in offline replay; trial44 extra1023 removed without removing previously confirmed correct IDs in40 final histories. These are offline checks, not native-success claims. Native11-episode stage regression plan is supervised by `osp-fix-sequence.service`, max2 benchmarks, videos on. Authoritative plan/state: `training_robust/native90/sequential_fixes_v10_v13/sequence.json`; implementation notes: sibling `implementation.json`. Holdout remains unused.
