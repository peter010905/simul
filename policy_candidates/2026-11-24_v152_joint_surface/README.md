# Mixed v1.5.2.14 · Joint Surface

Focus Continuity 실물 기록은 미관찰 벽을 예약하는 분기와 장애물 표면 탐색을 별도로 선택했다. 이 후보는 두 종류의 안전한 관찰점에 같은 보상과 경로비를 적용한다. 가까운 두 지점을 함께 갈 때의 이득은 여유 셀과 Nav2 비용지도에서 연결된 경우에만 계산한다. 예상 시야는 점수에만 쓰고 RGB 관찰 상태는 기존 카메라 파이프라인이 갱신한다.

같은 Focus Continuity 소스에서 별도 작업공간을 설치한다.

```sh
python3 policy_candidates/2026-11-24_v152_joint_surface/install.py \
  ~/v152_focus_continuity_box1122_ws ~/v15214_joint_surface_1124_ws
```

`replay.py`는 ROS Jazzy와 후보 작업공간을 불러온 뒤 실물 `run_logs/robot/<run-id>`에 실행할 수 있다. 목표 선택 당시 SLAM 지도, 카메라 확인 이력과 가능한 경우 비용지도, TF와 보정값을 불러온다. 복구, 마커 단서, 객체 추적 상태 전체를 되감지 않으므로 저장된 시점에서 선택이 달라지는지 확인하는 데 사용한다. 새 주행의 거리나 마커 발견률을 재생으로 추정하지 않는다.
