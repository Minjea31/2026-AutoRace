from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionTrainer
import torch
import argparse
from pathlib import Path


class NoHorizontalFlipTrainer(DetectionTrainer):
    def check_resume(self, overrides):
        super().check_resume(overrides)
        # Resuming restores checkpoint settings, so disable flipping afterwards too.
        self.args.fliplr = 0.0
        self.args.flipud = 0.0


parser = argparse.ArgumentParser()
parser.add_argument('--name', type=str, default='baseline')
parser.add_argument('--bs', type=int, default=8)
parser.add_argument('--epoch', type=int, default=100)
parser.add_argument('--patience', type=int, default=100,
                    help='Early stopping patience; 0 disables early stopping.')
parser.add_argument('--model_pt', type=str, default=str(Path(__file__).resolve().parents[1] / 'yolo26n.pt'))
parser.add_argument('--resume', action='store_true')
parser.add_argument('--data', type=str, default=str(Path(__file__).resolve().parent / "dataset.yaml"))
parser.add_argument("--device", type=str, default='0')

args = parser.parse_args()

project_dir = Path(__file__).resolve().parent / "checkpoints"
model = YOLO(args.model_pt)
model.train(data=args.data, epochs=args.epoch, imgsz=640, device=args.device, name=args.name,
            batch=args.bs, workers=4, save_period=5, project=str(project_dir),
            trainer=NoHorizontalFlipTrainer, fliplr=0.0, flipud=0.0,
            patience=args.patience, resume=args.resume)
