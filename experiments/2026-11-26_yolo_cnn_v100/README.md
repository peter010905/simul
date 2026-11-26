# V100 마커 검출: YOLO와 CNN, 기본형과 증강형

2026-11-26 오프라인 후보 실험. 실물 로봇이나 대시보드에 적용하지 않았다. 네 모델은 **마커 위치 후보**만 출력한다. ArUco ID, 실제 접근 거리, 지도 좌표는 이 실험의 정답이나 출력이 아니다.

## 비교 설계

| 모델 | 기본형 | 증강형 |
|---|---|---|
| YOLO11n | 640×480 전체 프레임 10,000장 | 같은 원본 + 변형 프레임 5,000장 |
| CNN | 동일 프레임에서 추출한 영역 패치 | 동일 원본 영역 + 변형 프레임 영역 |

- 데이터: `vanessabajaj/aruco-synthetic-detection`, 리비전 `7506a466db549c3ca1e187299316196acca2819c`. COCO의 640×480 이미지와 상자 라벨만 사용한다.
- 분할: 공급된 `train2017` 10,000장 학습, `val2017` 앞 500장 모델·임계값 보정, 나머지 1,000장 최종 평가. 네 모델 모두 동일한 분할이다. 공급 데이터의 배경 장면·마커 ID 수준 독립성은 보장되지 않는다.
- 추가 증강: 학습 이미지 중 절반을 한 번 더 저장한다. 카메라 화면 전체에 최대 ±28° 회전, 0.90–1.08배 크기 및 네 모서리 원근 변형(폭·높이의 ±5.5%)을 적용한다. 대부분에 가우시안 σ 0.45–1.55, 일부에 3/5픽셀 방향 블러를 준다. COCO 상자를 동일한 호모그래피로 변환한다. 검증·평가 영상은 변형하지 않는다.
- YOLO: 사전학습 `yolo11n.pt`, Ultralytics 8.4.219, 640 입력, 20 epoch 요청, GPU 0/1. 내장 회전·원근·색상·모자이크 증강은 두 실행 모두 꺼서 추가 프레임의 효과를 분리한다.
- CNN: 65,073개 파라미터의 기존 `CandidateCNN` 구조를 처음부터 학습한다. 전역/국소 threshold의 느슨한 윤곽이 만든 48×48 패치 분류기다. 매 배치 양성·음성을 반씩 뽑고 두 실행에 같은 업데이트 횟수를 준다. 윤곽을 만들지 못하면 CNN 분류기가 아무리 좋아도 전체 프레임 검출은 실패한다.
- 첫 CNN은 패치 검증 F1이 높아도 전체 프레임에서 줄무늬와 글자를 많이 오인했다. **학습 영상** 3,000장에서 모델별 고득점 음성 패치를 최대 4개씩 채굴했다. 후보 패치가 어느 주석 마커와도 겹치지 않는 경우만 음성으로 썼다. 기본형 6,665개, 증강형 8,249개를 모아 각각 같은 2,500번의 업데이트로 추가 학습했다. 보정 분할에서 추가 학습 전후의 전체 프레임 성능을 비교하여 최종 CNN을 선택한다.
- 평가는 **전체 프레임**에서 상자 IoU≥0.50의 1:1 대응을 사용한다. 500장 보정 분할에서 신뢰도 임계값을 F1로 선택한 후 1,000장 평가 분할에 고정한다. 전체 precision·recall·F1과 `blur`, `rotation_perspective` 태그별 recall, 마커가 없는 프레임의 오검출, 처리 시간을 보고한다.
- 합성 데이터 결과와 실물 로봇 성능을 혼동하지 않는다. 네 모델의 학습 영상 수가 다르므로 YOLO의 epoch 수가 같아도 증강형의 총 업데이트 수가 많다. CNN은 업데이트 수를 맞춘다.

## 재현

서버의 독립 환경과 데이터 위치: `/home/adminuser/marker_yolo_cnn_20261126`. `download.py`는 공개 데이터를 고정 리비전에서 받는다. 독립 Python 환경은 CUDA PyTorch 2.7.1, OpenCV 4.7.0, NumPy 1.26.4와 Ultralytics 8.4.219를 사용했다.

```bash
python download.py --output data
python prepare.py --data data --output prepared
python train_yolo.py --prepared prepared --output runs --variant plain --device 0 --epochs 20
python train_yolo.py --prepared prepared --output runs --variant augmented --device 1 --epochs 20
python train_cnn.py --prepared prepared --output runs --candidate . --stage prepare
python train_cnn.py --prepared prepared --output runs --candidate . --stage train --variant plain --device 2
python train_cnn.py --prepared prepared --output runs --candidate . --stage train --variant augmented --device 2
python mine_cnn.py --prepared prepared --weights runs/cnn/plain/best.pth --candidate . --output runs/cnn-hard/plain.npz --variant plain --device 2 --frames 3000
python mine_cnn.py --prepared prepared --weights runs/cnn/augmented/best.pth --candidate . --output runs/cnn-hard/augmented.npz --variant augmented --device 3 --frames 3000
python finetune_cnn.py --runs runs --candidate . --variant plain --device 2
python finetune_cnn.py --runs runs --candidate . --variant augmented --device 3
```

`candidate.py`는 기존 기하 필터 없는 CNN 후보의 정확한 구조와 제안 함수 사본이다.

## 1,000장 예약 평가 결과

정답 마커 상자 991개, 마커가 없는 이미지 134장. 모든 임계값과 체크포인트는 앞의 500장 보정 분할에서 선택했다. `TP/FP/FN`은 IoU≥0.50에서 일대일 대응한 상자 수다. `흐림`과 `회전·원근`은 데이터셋이 제공한 난도 태그를 가진 정답 상자의 recall이다.

| 모델 | 임계값 | TP / FP / FN | Precision | Recall | F1 | 흐림 recall | 회전·원근 recall | 프레임 중앙 처리 시간 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| YOLO 기본형 | 0.45 | 896 / 37 / 95 | 96.0% | **90.4%** | **93.1%** | **89.9%** | **91.5%** | 16.2 ms |
| YOLO 증강형 | 0.35 | 878 / 22 / 113 | **97.6%** | 88.6% | 92.9% | 88.3% | 88.6% | 15.1 ms |
| CNN 기본형 + 어려운 음성 | 0.95 | 696 / 244 / 295 | 74.0% | 70.2% | 72.1% | 67.1% | 69.5% | 71.6 ms |
| CNN 증강형 + 어려운 음성 | 0.98 | 657 / 143 / 334 | 82.1% | 66.3% | 73.4% | 61.7% | 64.8% | 78.4 ms |

YOLO 기본형의 F1이 가장 높았다. YOLO 증강형은 오검출이 37→22개로 줄었지만 정답 검출도 896→878개로 감소했다. 동일 프레임 부트스트랩에서 증강형−기본형 F1 차이는 −0.0028, 95% 구간 [−0.0124, 0.0066]이다. 이 데이터에서 증강형의 총괄 개선을 주장할 수 없다. CNN 증강형−기본형 F1 차이도 +0.0128, 95% 구간 [−0.0017, 0.0259]로 불확실하다. 반면 YOLO 기본형−가장 나은 CNN의 F1 차이는 +0.1977, 95% 구간 [0.1753, 0.2214]였다. 부트스트랩은 이 합성 평가 이미지의 프레임 재표본만 나타낸다.

CNN 원래 학습본은 보정 세트의 전체 프레임 F1이 기본형 0.661, 증강형 0.681이었다. 학습 프레임의 어려운 음성을 추가한 최종본은 각각 0.722, 0.736으로 올랐다. 그러나 느슨한 윤곽 경로는 학습 마커 9,983개 중 9,023개, 보정 마커 495개 중 440개에서만 IoU≥0.35 후보를 만들었다. CNN이 처음부터 볼 수 없는 마커가 있다.

YOLO는 COCO 사전학습에서 시작하고 CNN은 무작위 초기화에서 시작했으므로 이 결과는 **실제 구성한 두 후보 경로**의 비교이며 순수한 신경망 구조 비교가 아니다. 처리 시간은 두 CNN 예약 평가를 병행한 실행에서 측정해 엄밀한 단독 지연 비교가 아니다. 데이터는 합성 마커를 배경 영상에 넣은 공개 자료다. 독립 실물 환경의 검출 성능, 마커 ID, 1.5 m 접근, 지도 좌표, 충돌 없는 주행을 이 실험으로 증명하지 않았다.

결과 원본은 [`artifacts/comparison.json`](artifacts/comparison.json)에 있다. 선택된 가중치는 `artifacts/weights/` 아래 네 파일로 보존했다. `infer_yolo.py`는 선택된 YOLO 기본형으로 이미지의 마커 상자·점수와 미리보기를 만들며 ID는 `null`이다.

```bash
python infer_yolo.py --weights artifacts/weights/yolo-plain.pt --input image.jpg --output predictions --device 0 --confidence 0.45
```

## 실물 카메라 영상 로컬 재생

이전 검토에 쓰인 250×250 207장과 640×480 50장을 Mac CPU에서 네 모델에 동일하게 입력했다. 검토된 양성 영역에서 한 마커의 일부가 중복으로 표시된 상자 두 개를 제외한 22개와 기존 ArUco Original 검출 후 육안으로 확인된 175개 양성 영역을 사용했다. 총 197개 정답 중 148개가 ID 7이며 연속 프레임이 포함된다. 비스듬한 표본은 정답 마커 사각형의 최단 변이 최장 변의 65% 이하인 16개다. 그중 15개가 ID 42다. 흐림 표본 59개는 모델 결과를 보기 전에 마커 확대본에서 경계 번짐이나 잔상이 보이는 경우로 표시했다. 두 조건에 모두 해당하는 표본은 3개다.

| 모델 | 고정 신뢰도 | 전체 검출 | 비스듬함 | 흐림 |
|---|---:|---:|---:|---:|
| YOLO 기본형 | 0.45 | 0/197 | 0/16 | 0/59 |
| YOLO 증강형 | 0.35 | 6/197 | 0/16 | 0/59 |
| CNN 기본형 | 0.95 | 0/197 | 0/16 | 0/59 |
| CNN 증강형 | 0.98 | 0/197 | 0/16 | 0/59 |

전체 프레임에서 IoU 0.50 이상의 상자만 맞은 것으로 집계했다. 위 신뢰도는 합성 보정에서 확정한 수치다. YOLO 증강형이 잡은 6개는 모두 ID 7이며, 이 모델은 257장에 총 49개 후보를 냈다. 네 모델은 같은 Mac 환경에서 공개 합성 제어 이미지에 후보를 출력했고, YOLO 기본형의 두 점수는 V100 실행과 소수 셋째 자리까지 일치했다.

사후 진단에서 임계값을 0.05로 낮추면 YOLO 증강형이 전체 23개와 흐린 마커 2개를 찾으면서 총 161개 후보를 냈다. CNN 증강형은 전체 65개, 비스듬한 마커 5개, 흐린 마커 3개를 찾았지만 총 후보가 2,968개였다. 검토된 비마커 영역 68개 중 23개에도 겹쳤다. 실물 영상의 모든 마커와 배경에 라벨이 있는 것은 아니므로, 해당 후보 총수는 오탐 수가 아니며 전체 precision이나 F1은 산출하지 않았다. 이 영상에서 네 모델 모두 기존 기준으로는 비스듬하거나 흐린 마커를 안정적으로 찾지 못했다.

[집계 원본](artifacts/real-replay.json)에는 촬영 조건별 결과, 낮은 신뢰도 진단 및 합성 제어 결과가 있다. 원본 영상과 프레임별 출력은 로컬에 보관했다. 기존 `geometry-free-candidates-r3`의 라벨 파일과 이 폴더의 흐림 라벨로 재현할 수 있다.

```bash
python replay_real.py \
  --reviewed-regions "$review_root/reviewed-test.json" \
  --positive-pool "$review_root/real-positive-pool.json" \
  --blur-labels real_blur_labels.json --output "$results_dir"
```

## 후속 관찰 후보로 평가

같은 실물 257장을 대상으로, 후보가 마커 상자 면적의 80% 이상을 포함하고 후보 면적이 정답의 6배 이하이면 회수로 계산했다. 흰 종이까지 포함한 상자도 인정한다. 모델별로 신뢰도 대신 프레임 내 순위로 최대 1개, 3개, 5개를 남겼다.

| 모델 | 상위 1개 전체 | 상위 3개 전체 | 상위 5개 전체 | 상위 5개 비스듬함 | 상위 5개 흐림 |
|---|---:|---:|---:|---:|---:|
| YOLO 기본형 | 63/197 | 74/197 | 77/197 | 9/16 | 8/59 |
| YOLO 증강형 | 122/197 | 142/197 | 144/197 | 8/16 | 27/59 |
| CNN 기본형 | 149/197 | 167/197 | 173/197 | 1/16 | 55/59 |
| CNN 증강형 | 158/197 | 181/197 | 188/197 | 11/16 | 55/59 |

CNN 증강형의 상위 5개는 257장에 후보 1,285개를 냈고 검토된 비마커 영역 68개 중 5개를 포함했다. 전체 배경에 정답이 없어 오탐률로 해석할 수 없다. 후보 면적 상한을 4배로 줄이면 CNN 증강형은 전체 176/197과 비스듬함 6/16을 회수해, 비스듬한 마커 회수율은 허용하는 후보 크기에 민감했다. 이 기준은 후속 관찰에 넘길 영역을 평가한다. ID 판독이나 실제 관찰 지점으로 이동했을 때의 성공률은 별도 측정이 필요하다.

재현할 때 위의 `replay_real.py` 명령에 `--cnn-score-floor 0`을 추가해서 CNN의 낮은 점수 후보까지 저장한다. 다음 명령으로 같은 프레임당 후보 수를 적용할 수 있다. [전체 집계](artifacts/real-candidate-replay.json)

```bash
python score_candidates.py --predictions "$results_dir/predictions.jsonl" --output "$results_dir/candidates.json"
```
