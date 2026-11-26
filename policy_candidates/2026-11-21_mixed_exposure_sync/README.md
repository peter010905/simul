# Mixed v1.5.1 · 촬영 시각 센서 동기화

v1.5.0의 RGB 대기는 개선됐지만, 대기 중 `self.scan`과 `self.depth`가 최신 메시지로 바뀌면 영상의 촬영 시각과 범위 센서가 어긋날 수 있다. 새 판은 작은 유한 버퍼에서 촬영 시각에 맞는 샘플을 선택한다. 원본 v1.2와 v1.5.0은 보존한다.

- LiDAR는 RGB보다 늦지 않은 가장 가까운 샘플을 선택한다. 기존 250ms 차이 한도는 유지한다. `lookup_transform_full`로 LiDAR 촬영 시각과 RGB 촬영 시각을 odom에서 연결해 로봇 이동을 반영한다.
- 깊이 영상은 같은 optical frame·크기·지원 encoding인 등록 영상에서 가장 가까운 노출을 고른다. 기존 150ms 한도를 유지한다. 시각을 새로 쓰거나 최신 TF로 과거 좌표를 덮지 않는다.
- scan 64개·depth 8개만 보존한다. 센서가 없거나 조건에 맞지 않으면 거리 없음으로 남긴다. 마커의 위치·근거리 증거를 임의 보완하지 않는다.
- camera queue의 `dispatched`와 유효 processed_camera FPS를 구분한다.

합성 ROS 영상에서 TF가 250ms 늦을 때 v1.5.0은 37프레임 판독·거리 없음 37회·등록 0개, 동기화판은 ID42 등록·오류 0회였다. 이 동기화 코드는 v1.6.0에서도 byte-identical하게 사용한다. 실제 로봇이나 전체 주행 성공을 뜻하지 않는다.

모든 수정판의 실제 ROS·시간/프레임 정합·이동 보정·안전 우선순위·기존 검출 검사는 44개를 통과했다. [반례·동결 소스 해시](source_manifest.json)와 [주행 비교](../../experiments/2026-11-21_camera_aware_policy/README.md)를 함께 확인한다. 새 native 14회 모두 엄격 기준을 통과했고 범퍼 접촉은 0회였다. 선정 뒤 처음 사용한 마지막 세 맵에서도 모두 통과했다. 실물 기본값은 바꾸지 않는다.

동결 스테이지: `flow_v151_exposure_sync_r2_ws`. detail(640×640·5FPS)을 먼저 비교했고 balanced(480×480·5FPS), glance(480×480·10FPS)도 같은 동결 알고리즘에서 재검증했다. 시간·성공·전송량 기준으로 **balanced**를 시뮬 기본 선택으로 정했다. `./sim-camera.sh v1.5.1-detail 112201`로 시뮬에서 선택한다. 공통 비교 lock이 사용 중이면 새 실행을 막는다.

`overlay/osp_mission`에는 실제 실행한 다섯 소스 파일의 스냅샷이 있다. `install_snapshot.py`는 원본 v1.2의 나머지 218파일과 오버레이 SHA-256을 검사한 뒤 새 경로에 동일 소스를 구성한다. 기존의 v1.5.0에서 이어 설치하는 `install.py`도 보존한다.

[독립 릴리스 스냅샷](../../policy_releases/2026-11-21_mixed_camera_sync_v151/README.md)에는 실제 실행한 텍스트 221개와 외부 설정이 있어 이전 VM 없이도 빌드할 수 있다. 새 Jazzy 경로에서 두 네이티브 바이너리까지 동결 VM과 동일하게 재현했다.
