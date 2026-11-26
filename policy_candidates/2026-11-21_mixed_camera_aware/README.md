# Mixed v1.5.0 · 촬영 시각 기반 카메라 처리

이번 순차 비교는 42회까지 완료했고, 최종 시뮬 선택은 [v1.5.1-balanced 동기화판](../2026-11-21_mixed_exposure_sync/README.md)이다. 이 폴더의 v1.5.0과 v1.2도 그대로 보존한다. [모든 회차와 한계](../../experiments/2026-11-21_camera_aware_policy/README.md).

v1.2를 유지하면서 해상도·FPS 세 프로필을 각각 별도 버전으로 만든다. 새 v1.5.0은 TF 대기 때문에 읽은 마커를 버리는 문제, 한 마커 예외로 이웃 ID까지 놓치는 문제, FPS와 맞지 않던 다중 시점 기록을 보완한다. [코드 검토와 변경 경계](REVIEW.md).

| 프로필 | 기준판 | 개선판 | 카메라 | 순수 RGB payload |
|---|---|---|---|---:|
| balanced | v1.2-balanced | v1.5.0-balanced | 480×480·5FPS | 27.6Mbps |
| glance | v1.2-glance | v1.5.0-glance | 480×480·10FPS | 55.3Mbps |
| detail | v1.2-detail | v1.5.0-detail | 640×640·5FPS | 49.2Mbps |

모두 시뮬 선택 버전이며 실물 기본값은 유지한다. 480×480은 일반 640×480인 480p와 다르다. [프로필](profiles.json), [각 버전 manifest](versions/).

## 확인한 결함과 검사

- 촬영 시각의 map/odom TF를 기다릴 때 executor를 막지 않는다. 최신 입력 한 장만 계속 덮으면 TF 지연이 프레임 간격보다 클 때 기아가 생기므로, 최대 8장·0.4초 이내에서 TF가 준비된 최신 노출을 고른다.
- ID별 실패를 분리하고 전체 처리 뒤 heartbeat를 보낸다. 촬영 age·callback wall p95·TF 대기·폐기 횟수를 별도 기록한다. 이 queue의 processed 통계는 선택한 작업 수이며 실제 유효 영상률은 processed_camera 토픽으로 측정한다.
- 기존 250×250 OAK-D fingerprint에만 있던 수평 광학 보정을 LiDAR·삼각측량·관찰 기하에 일관되게 한 번 적용한다. 다른 해상도의 실물 보정 계수를 임의 추정하지 않는다.
- 각 노출의 실제 자세를 사용하는 다중 시점 관측에서 5FPS 이동 창과 60초 keyframe 보존을 맞춘다. 최신 시점쌍이 부적합하면 더 오래된 유효 시점도 살핀다.
- Original 패턴·18 native px·세 프레임 합의·이웃 ID 분리·1.3m 근거리 영상 증거는 그대로다. 목표 선택·Nav2·속도·안전 여유도 그대로다. [218개 원본 일치 파일](source_manifest.json).

ROS 실제 콜백, TF 예외 격리, 광학 보정, 버퍼 기아·역행·만료, 5FPS 이동·오래된 시점 유지, 기존 검출과 15개 센서 조건을 포함한 검사 **30개 통과**. 합성 ROS 카메라의 TF를 120ms 늦춘 실험에서 기준판은 37프레임 판독에도 ID 등록 0개, 새 판은 ID42 등록·오류 0회였다. [기준 결과](tf_delay_baseline.json), [개선 결과](tf_delay_candidate.json). 이는 실제 로봇이나 Gazebo 주행 결과가 아니다.

## Gazebo 평가

[공통 조건과 최종 결과](../../experiments/2026-11-21_camera_aware_policy/README.md)에서 기존/개선 × 세 프로필을 비교한다. 선정용 2맵에서 총 12회를 먼저 수행하고, 선정에 사용하지 않은 3맵에서 두 정책의 선택 프로필을 다시 비교한다. 실물 다운 방지와 모든 환경의 전역 최적성을 증명하는 시험은 아니다.

성공은 세 ID 등록, 독립 실제 거리 각각 ≤1.5m, 기존 정책의 세 장 ≤1.3m 근거리 증거, 초기 좌표 정렬을 고정한 지도 오차 ≤0.25m, 범퍼 0회다. 초기 등록 ID, 실패·센서 중단·정지·회전·거리·자원 사용을 별도 저장한다.

## 한 버전씩 실행

준비된 `osp-jazzy` VM과 비교용 frozen 맵을 사용한다. 진행 중인 비교가 있으면 같은 lock이 실행을 막는다. 명령은 시뮬 전용 domain 174에서만 실행한다.

```sh
./sim-camera.sh v1.5.0-balanced 112101
./sim-camera.sh v1.5.0-glance 112102
./sim-camera.sh v1.5.0-detail 112103
# v1.2-balanced / v1.2-glance / v1.2-detail도 같은 방법
```

결과 카드는 [8780 시뮬 보드](http://192.168.252.2:8780/)에 저장된다. 이 쉘은 실물 보드·ROS domain 4를 사용하지 않는다.

## 재현 설치

Jazzy VM에서 보존 v1.2가 있는 새 경로로 설치한다. 실행 중인 스테이지에는 덮어쓰지 않는다.

```sh
python3 policy_candidates/2026-11-21_mixed_camera_aware/install.py \
  ~/flow_v3_r32_marker_relook_ws ~/new_camera_aware_ws
cd ~/new_camera_aware_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select osp_mission
```

실제 캠페인은 `flow_v15_camera_aware_r2_ws`를 동결해 사용했다. 첫 빌드에서 회귀 검사가 잡은 함수 내부 import 충돌을 수정한 스테이지다. 실패 빌드는 주행 비교에 사용하지 않았다.
