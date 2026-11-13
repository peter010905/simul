# 최신 Gazebo 검증 정책

정책 계수와 그 계수를 실행한 `osp_mission` 소스의 고정 스냅샷입니다. 정책은 `policy.json`, 파일별 SHA256과 검증 결과는 `release.json`에 있습니다.

이 버전은 직사각형 환경에서 **388.627초**, 정확한 ID 3개, 각 실제 1.5m 접근, 최종 지도 좌표 오차 0.5m 이내, 충돌 0회로 완주했습니다. 여러 공간의 독립 검증과 실제 로봇 검증은 아직 완료하지 않았습니다. 정책 파일의 원래 후보 상태 문구는 검증 때 사용한 그대로 보존했습니다.

ROS 2 Jazzy와 저장소의 의존성이 설치된 환경에서 새로운 작업 공간에 빌드할 수 있습니다.

```bash
python3 policy_releases/2026-11-13_face_intent_v2/build.py \
  --workspace "$PWD/training_robust/native90/released_face_intent_v2" --test
source training_robust/native90/released_face_intent_v2/install/setup.bash
```

빌드 스크립트는 파일 해시를 확인하고, 포함된 C++ 소스로 두 보조 프로그램을 컴파일한 뒤 패키지를 빌드합니다. `--test`는 런타임 테스트를 실행합니다. 시뮬레이터나 실제 로봇을 자동으로 시작하지 않습니다.

시뮬레이션 실행 시 정책 경로는 이 폴더의 `policy.json`을 지정합니다. 실행 중인 학습 공간과 구분되는 새 작업 공간을 사용해야 합니다.

[실제 검증 증거](../../repository_snapshot/face_intent_native_success.json)
