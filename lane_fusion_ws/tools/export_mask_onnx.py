"""Export the lane U-Net as a MASK model: normalisation, softmax and argmax inside.

  image  float32 [1,3,H,W]  RGB 0..255 (cv2.dnn.blobFromImage(bgr, 1, (W,H), swapRB=True))
  label  int32   [1,H,W]    argmax class (0 background, 1 orange, 2 white dashed)
  prob   float32 [1,H,W]    softmax probability of that class (node thresholds it)

On the Jetson this moves every per-pixel step except the threshold to the GPU.
Checks the result against the logits model + runtime.decode on the given frames.

usage (python with torch + onnxruntime):
  python tools/export_mask_onnx.py src/autorace_lane_fusion_deep/model/best.pt \
      src/autorace_lane_fusion_deep/model/lane_640_mask.onnx [--check IMG_DIR]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/autorace_lane_fusion_deep'))
from autorace_lane_fusion_deep.constants import MEAN, STD  # noqa: E402
from autorace_lane_fusion_deep.model import load_checkpoint  # noqa: E402
from autorace_lane_fusion_deep.runtime import blob, decode, preprocess  # noqa: E402


class MaskHead(nn.Module):
    def __init__(self, net):
        super().__init__()
        self.net = net
        self.register_buffer('scale', 1 / (255 * torch.tensor(STD).view(1, 3, 1, 1)))
        self.register_buffer('shift', (torch.tensor(MEAN) / torch.tensor(STD)).view(1, 3, 1, 1))

    def forward(self, image):
        logits = self.net(image * self.scale - self.shift).float()
        prob, label = torch.softmax(logits, dim=1).max(dim=1)
        return label.to(torch.int32), prob


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('weights', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--check', type=Path, help='directory of .png/.jpg frames for the equivalence check')
    a = p.parse_args()
    if a.output.exists():
        p.error(f'{a.output} exists')
    net, checkpoint = load_checkpoint(a.weights, torch.device('cpu'))
    net.eval()
    width, height = checkpoint['size']
    head = MaskHead(net).eval()
    dummy = torch.zeros(1, 3, height, width)
    torch.onnx.export(head, dummy, str(a.output), opset_version=17, input_names=['image'],
                      output_names=['label', 'prob'], do_constant_folding=True)

    import onnxruntime as ort
    session = ort.InferenceSession(str(a.output), providers=['CPUExecutionProvider'])
    frames = sorted(a.check.glob('*.png'))[:20] + sorted(a.check.glob('*.jpg'))[:20] if a.check else []
    rng = np.random.default_rng(0)
    images = [cv2.imread(str(f)) for f in frames] or [(rng.random((480, 640, 3)) * 255).astype(np.uint8)]
    worst = 0.0
    for bgr in images:
        with torch.inference_mode():
            reference = decode(net(torch.from_numpy(preprocess(bgr, width, height))).numpy(), (width, height))
        label, prob = session.run(['label', 'prob'], {'image': blob(bgr, width, height)})
        mask = label[0].astype(np.uint8)
        mask[prob[0] < 0.5] = 0
        worst = max(worst, float((mask != reference).mean()))
    print(f'exported {a.output} ({width}x{height}); mask disagreement vs logits model on '
          f'{len(images)} frame(s): max {worst:.6f}')


if __name__ == '__main__':
    main()
