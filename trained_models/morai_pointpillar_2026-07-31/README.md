# MORAI PointPillars 인계 문서

- 저장일: 2026-07-31 (Asia/Seoul)
- 상태: 학습 40 epoch 및 epoch 35~40 자동 평가 정상 완료
- 선택 모델: `checkpoint_epoch_39.pth`
- 바로 사용할 경로: `best_model.pth`
- 클래스: `Car` 1종

## 결론

일반적인 LiDAR 차량 검출·시각화 용도로 epoch 39를 최종 선택함. epoch 39는
BEV/3D, AP/AP_R40, IoU 0.7/0.5의 8개 지표 중 6개와 Recall@0.3/0.5에서
가장 높았으며, 8개 AP 지표의 동등 가중 평균도 80.8565로 최고였음.

epoch 37은 엄격한 3D AP_R40@0.7만 보면 74.0105로 더 높지만, 전체 검출
성능과 BEV 성능을 함께 고려해 epoch 39를 선택함. epoch 40은 strict 3D 성능과
Recall@0.7이 크게 떨어져 사용하지 않음.

## 보관 파일

- `best_model_epoch_39.pth`: 선택한 모델의 독립 복사본
- `best_model.pth`: 위 파일을 가리키는 상대 심볼릭 링크
- `pointpillar_morai.yaml`: 학습 당시 모델 설정
- `morai_dataset.yaml`: 학습 당시 데이터 설정
- `validation_10_rounds.json`: 학습 재개 직전 10회 검증 PASS 기록
- `evaluation/epoch_39_result.pkl`: epoch 39 검증 예측 결과
- `evaluation/training_and_evaluation.log`: 학습 및 epoch 35~40 평가 원본 로그
- `SHA256SUMS`: 보관 파일 무결성 확인값

원본 프로젝트 경로:

```text
/home/bisa/morai-mpc-agent-morai-lio-gps-integration
```

## 선택 모델 주요 성능

검증 세트 2,000 samples 기준임.

| 지표 | epoch 39 |
|---|---:|
| Recall@0.3 | 0.969707 |
| Recall@0.5 | 0.901708 |
| Recall@0.7 | 0.760232 |
| BEV AP_R40 @ IoU 0.7 | 77.1820 |
| 3D AP_R40 @ IoU 0.7 | 72.4844 |
| BEV AP_R40 @ IoU 0.5 | 88.9637 |
| 3D AP_R40 @ IoU 0.5 | 88.4316 |

## 환경 기록

- Python 3.8.10
- PyTorch 2.0.1+cu118
- CUDA 11.8
- OpenPCDet 0.6.0+233f849
- OpenPCDet commit `233f849829b6ac19afb8af8837a0246890908755`
- GPU: NVIDIA GeForce RTX 3090
- 학습 batch size: 2, workers: 0
- 데이터: train 8,005 / validation 2,000 samples

## 가장 먼저 할 검증

새 터미널에서 아래 명령을 그대로 실행함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration/trained_models/morai_pointpillar_2026-07-31
sha256sum -c SHA256SUMS
../../.venv-openpcdet/bin/python -c "import torch; p='best_model.pth'; x=torch.load(p, map_location='cpu'); print(x['epoch'], x['it'], len(x['model_state']))"
```

정상 결과는 checksum 전부 `OK`, 체크포인트 정보는 `39 144108 127`임.

## 선택 모델 재평가

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration/third_party/OpenPCDet/tools
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH:-}
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
../../../.venv-openpcdet/bin/python test.py \
  --cfg_file ../../../training/openpcdet/pointpillar_morai.yaml \
  --batch_size 2 \
  --workers 0 \
  --ckpt ../../../trained_models/morai_pointpillar_2026-07-31/best_model.pth
```

## 추가 학습을 재개할 때

현재 40 epoch 학습은 이미 끝났고 epoch 40에서 성능 저하가 확인됐으므로, 같은
데이터로 단순히 epoch 수만 늘리는 것은 권장하지 않음. 데이터를 추가하거나 설정을
바꾼 경우에만 아래 순서로 epoch 39에서 새 실험을 시작함.

1. `pointpillar_morai.yaml`의 `NUM_EPOCHS`를 40보다 큰 목표 총 epoch로 변경함.
2. 하드웨어 보호 설정을 적용함.
3. CUDA forward/backward와 체크포인트를 10회 검증함.
4. 기존 결과를 덮어쓰지 않도록 새 `extra_tag`로 학습함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
training/openpcdet/apply_hardware_guard.sh
.venv-openpcdet/bin/python training/openpcdet/validate_training_stack.py \
  --rounds 10 \
  --batch-size 2 \
  --allow-unsupported-bios

cd third_party/OpenPCDet/tools
export CUDA_VISIBLE_DEVICES=0
export CUDA_MODULE_LOADING=LAZY
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH:-}
../../../.venv-openpcdet/bin/python train.py \
  --cfg_file ../../../trained_models/morai_pointpillar_2026-07-31/pointpillar_morai.yaml \
  --extra_tag morai_continue_from_epoch39 \
  --batch_size 2 \
  --workers 0 \
  --fix_random_seed \
  --num_epochs_to_eval 5 \
  --ckpt ../../../trained_models/morai_pointpillar_2026-07-31/best_model.pth \
  --ckpt_save_interval 1 \
  --ckpt_save_time_interval 180 \
  --max_ckpt_save_num 30 \
  --wo_gpu_stat
```

주의: 보관 폴더의 YAML을 직접 수정하기보다는 복사본을 만들어 새 실험에 사용해야
현재 모델의 재현 기록이 보존됨.

## 다음 권장 작업: 실시간 LiDAR 연결

현재 모델 학습과 오프라인 평가는 완료됐지만 ROS 실시간 추론 노드는 아직 없음.
다음 작업은 아래 순서로 진행함.

1. MORAI LiDAR `sensor_msgs/PointCloud2` 토픽을 확인함.
2. x/y/z/intensity를 OpenPCDet 입력으로 변환하는 ROS 노드를 작성함.
3. 이 폴더의 `best_model.pth`를 로드해 PointPillars 추론을 수행함.
4. 차량 3D 박스와 신뢰도를 `visualization_msgs/MarkerArray`로 발행함.
5. RViz에서 점군과 예측 박스의 좌표·yaw·크기 정렬을 검증함.
6. 실제 주행 전에 거리 구간별 검출률, 지연 시간, 오검출을 반복 검증함.

다음 작업을 요청할 때 아래 문장을 그대로 사용하면 됨.

```text
/home/bisa/morai-mpc-agent-morai-lio-gps-integration/trained_models/morai_pointpillar_2026-07-31/README.md를 읽고, best_model.pth를 사용하는 ROS 실시간 LiDAR 차량 검출 노드와 RViz 시각화를 이어서 구현하고 검증해 줘.
```
