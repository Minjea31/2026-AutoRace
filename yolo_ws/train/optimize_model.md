# 모델 최적화 실행

```bash
conda activate dl_env
cd ~/2026-AutoRace/yolo_ws/train
```

기본 모델 `../model/best.pt` 최적화:

```bash
python optimize_model.py
```

`../model/새모델.pt` 최적화:

```bash
python optimize_model.py 새모델.pt
```

결과: `../model/새모델_fp16.engine`, `새모델_fp16.onnx`, `새모델_fp16.optimization.json`

출력 이름 지정:

```bash
python optimize_model.py 새모델.pt --output ../model/새모델_optimized.engine
```

경로 확인만 하기:

```bash
python optimize_model.py 새모델.pt --dry-run
```

ONNX까지만 생성하기:

```bash
python optimize_model.py 새모델.pt --onnx-only
```

기존 최적화 결과를 백업하고 다시 생성하기:

```bash
python optimize_model.py 새모델.pt --overwrite
```

생성한 엔진으로 YOLO 노드 실행:

```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/yolo_ws/install/setup.bash
export PYTHONPATH="$HOME/2026-AutoRace/yolo_ws/yolo26:${PYTHONPATH:-}"
ros2 run yolo_detect yolo_detect --ros-args \
  -p model_path:=$HOME/2026-AutoRace/yolo_ws/model/새모델_fp16.engine
```

도움말:

```bash
python optimize_model.py --help
```
