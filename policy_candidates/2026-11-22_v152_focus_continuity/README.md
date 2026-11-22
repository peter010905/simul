# Mixed v1.5.2.1 Focus Continuity

8780 상자 시험장의 첫 주행에서 Visual Survey는 벽 기둥의 43번을 46프레임에서 읽었다. 시뮬 시계 564.0초에 43번을 정면으로 다시 볼 목표를 선택했고 565.1초에 같은 번호가 카메라에서 다시 들어오자 그 목표를 취소했다. 가장 가까웠을 때 거리는 1.10m였다. 1.5m 안에서 마커 중심이 화면에 들어온 기록은 0프레임이었다. [원본 로그에서 추출한 사건과 영상 측정값](../../experiments/2026-11-22_random_exam/box-only/self-cancel-evidence.json)

이 버전은 특정 번호의 재관찰이나 접근을 이미 수행하는 동안 같은 번호의 유효한 영상이 들어오면, 위치 추정에 새 관측을 반영하고 현재 접근을 이어간다. 다른 마커가 나타날 때의 재평가, 범퍼·스캔 안전 처리, Nav2 경로 계획, ID 확정 및 근거리 증명은 상위 버전의 구현을 사용한다. 번호와 부착 위치에 관계없이 같은 조건을 적용한다.

팀용 Jazzy 시뮬레이터에서 별도의 작업 공간에 설치할 수 있다.

```bash
./team-sim.sh setup ~/mixed_focus_sim_ws --policy visual-survey-focus
./team-sim.sh run ~/mixed_focus_sim_ws --random-layout --tables fixed --markers three
```

카메라는 480×480, 5fps이고 주행은 최대 600초다. 첫 지도를 개발 사례로 구분하고 다른 네 지도에서 비교하도록 등록했다. 이후 분산 상자 세 회차로 실험을 전환했다. 기록된 마커 수, 등록 후 1.5m 접근, 벽·상자의 관찰 면적, 반복 이동, 충돌을 각각 측정한다. [등록한 비교 조건](../../experiments/2026-11-22_random_exam/focus-enrollment.json)

분산 배치 세 회차에서 상자 접근은 6/6장, 벽 접근은 2/3장이었다. [세 주행의 지도·영상과 미검출 프레임 분석](../../experiments/2026-11-22_random_exam/spread-three/RESULTS.md)
