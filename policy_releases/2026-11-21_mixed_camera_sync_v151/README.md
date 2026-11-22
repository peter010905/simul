# Mixed v1.5.1 · 독립 소스 스냅샷

실제 비교에 사용한 `flow_v151_exposure_sync_r2_ws`의 텍스트 파일 221개를 그대로 보존했다. 이전 VM의 v1.2 스테이지나 여러 후보 설치 스크립트를 차례로 실행하지 않아도 동일 소스를 구성할 수 있다. 알고리즘 설명과 완료 여부는 [비교 보고서](../../experiments/2026-11-21_camera_aware_policy/README.md)를 따른다.

`source_manifest.json`으로 모든 텍스트와 실제 사용한 외부 Nav2 설정·정책 가중치를 검증한다. 검사한 런타임 223개 파일 중 컴파일 결과 두 개(`_raycast.so`, `export_slam_graph_poses`)는 Git에서 제외하고 원래 C++ 소스로 다시 빌드한다. 다른 CPU·컴파일러에서는 바이너리 해시가 달라질 수 있다.

Ubuntu 24.04·ROS2 Jazzy에서 필요한 ROS·Nav2·SLAM Toolbox 개발 의존성이 준비돼 있을 때:

```sh
bash policy_releases/2026-11-21_mixed_camera_sync_v151/build.sh ~/new_camera_sync_ws
```

기존 디렉터리는 덮어쓰지 않는다. 빌드만 수행하며 ROS 노드나 로봇은 시작하지 않는다. 원래 텍스트의 해시, 컴파일 결과와 동결 VM 바이너리의 일치 여부를 새 워크스페이스의 `snapshot_build.json`에 저장한다.

현재 준비된 Mac·VM에서 검증된 시뮬 버전을 실행하는 진입점은 저장소 루트의 `./sim-camera.sh`다. 실물 기본 설정을 변경하거나 실물에서 검증된 릴리스라는 뜻은 아니다. 카메라 프로필은 [버전 manifest](../../policy_candidates/2026-11-21_mixed_camera_aware/versions/)에 분리했다.
