# MORAI PointPillars training

현재 단계:

1. `datasets/morai_lidar`에 차량 전용 원본 데이터를 수집함.
2. 수집 종료 후 `prepare_dataset.py`로 OpenPCDet 레이아웃을 만듦.
   이때 실제 크기의 3D 박스 내부 포인트를 다시 계산하고 5포인트
   미만인 희박한 박스는 학습 정답에서 제외함.
3. 시간 블록 단위 train/val 분할 후 info와 GT database를 생성함.
4. PointPillars를 학습하고 KITTI 3D AP 및 Recall을 평가함.

## 안정성 전제

- 메인보드 `TUF GAMING B660M-PLUS D4`에서 `i9-14900KS`를 사용할 때
  BIOS 2603 이상이 필수임.
- 장시간 학습은 `morai-hardware-guard.service`가 CPU 터보를 끄고 RTX
  3090 전력 제한을 250W로 적용한 상태에서 실행함.
- `validate_training_stack.py`가 실제 데이터, 모델, CUDA 연산, backward,
  optimizer, 체크포인트를 10회 모두 검증해야 학습을 시작함.
- 실행 중 워치독이 5초마다 온도, 메모리, 디스크, 커널 하드웨어 오류를
  확인하며 위험 징후가 있으면 PC가 아니라 학습 프로세스만 중단함.

## 실행 순서

```bash
python3 training/openpcdet/prepare_dataset.py
.venv-openpcdet/bin/python training/openpcdet/create_infos.py
cd third_party/OpenPCDet/tools
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH:-}
export OMP_NUM_THREADS=8
export MKL_NUM_THREADS=8
../../../.venv-openpcdet/bin/python train.py \
  --cfg_file ../../../training/openpcdet/pointpillar_morai.yaml \
  --extra_tag morai_10k \
  --batch_size 2 \
  --workers 0 \
  --fix_random_seed \
  --num_epochs_to_eval 5 \
  --wo_gpu_stat
```

검증 후 안전하게 재개:

```bash
training/openpcdet/validate_and_resume.sh
```

모델 입력:

- LiDAR `x`, `y`, `z`, `intensity`
- 범위: x/y ±70m, z -3~3m

모델 출력:

- `Car` 신뢰도
- LiDAR 좌표계 3D 중심 x/y/z
- 차량 길이/너비/높이
- 차량 yaw

학습 중 확인할 항목:

- classification, localization, direction loss
- validation 3D AP
- Recall@0.3/0.5/0.7 IoU
- 거리 구간별 검출률과 실제 포인트클라우드 예측 박스 정렬
