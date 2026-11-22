#!/usr/bin/env python3
"""Export a trained lane .pth/.pt checkpoint to ONNX and Jetson TensorRT FP16.

Examples (run in a Python environment with torch, torchvision, onnx, onnxruntime):
  python tools/optimize_lane_model.py /path/to/best.pth --export-only
  python tools/optimize_lane_model.py /path/to/best.pth --images /path/to/frames
  python tools/optimize_lane_model.py /path/to/best.pth --dry-run

Default architecture: this repository's LaneUNet (MobileNetV3-Small U-Net).
Accepts model/state_dict/model_state_dict dictionaries, or a bare state_dict.
A .pth extension alone does not identify an architecture: use --factory module:callable
for a different three-class segmentation model. Input is RGB 0..255 float32 NCHW;
normalization and label/probability decoding are embedded in the exported graph.
Classes: 0=background, 1=yellow/orange lane, 2=white lane. Size is never reduced.
Build engines on the target Jetson. Existing weights/engines and ROS nodes are untouched.
"""
import argparse
import datetime
import importlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('weights', type=Path)
    p.add_argument('--output-dir', type=Path, help='new output directory; must not already exist')
    p.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument('--factory', help='alternate model factory, e.g. my_lane:make_model; must return logits [1,3,H,W]')
    p.add_argument('--width', type=int, help='override input width (must also supply height)')
    p.add_argument('--height', type=int, help='override input height (must also supply width)')
    p.add_argument('--images', type=Path, help='representative .png/.jpg/.jpeg directory for equivalence checks')
    p.add_argument('--samples', type=int, default=20)
    p.add_argument('--threshold', type=float, default=0.5)
    p.add_argument('--max-disagreement', type=float, default=0.005, help='maximum fraction of disagreeing mask pixels')
    p.add_argument('--workspace-mib', type=int, default=512)
    p.add_argument('--build-level', type=int, choices=range(6), default=3)
    p.add_argument('--benchmark-seconds', type=int, default=5)
    p.add_argument('--trtexec', type=Path, help='default: PATH or /usr/src/tensorrt/bin/trtexec (TensorRT 10.x)')
    p.add_argument('--check-engine', action='store_true', help='also compare engine masks; needs Python TensorRT + CUDA')
    p.add_argument('--export-only', action='store_true', help='create and verify ONNX without building an engine')
    p.add_argument('--dry-run', action='store_true', help='print the plan without imports, writes, or GPU execution')
    a = p.parse_args(argv)
    a.weights = a.weights.expanduser().resolve()
    a.repo_root = a.repo_root.expanduser().resolve()
    if not a.weights.is_file():
        p.error(f'checkpoint not found: {a.weights}')
    if (a.width is None) != (a.height is None) or any(v is not None and v <= 0 for v in (a.width, a.height)):
        p.error('supply positive --width and --height together')
    if min(a.samples, a.workspace_mib, a.benchmark_seconds) <= 0:
        p.error('samples, workspace-mib and benchmark-seconds must be positive')
    if not 0 <= a.threshold <= 1 or not 0 <= a.max_disagreement <= 1:
        p.error('threshold and max-disagreement must be in [0,1]')
    if a.export_only and a.check_engine:
        p.error('--check-engine requires an engine build')
    if a.images is not None:
        a.images = a.images.expanduser().resolve()
        if not a.images.is_dir():
            p.error(f'image directory not found: {a.images}')
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    a.output_dir = (a.output_dir or a.weights.parent / 'optimized' / f'{a.weights.stem}-{stamp}').expanduser().resolve()
    if a.output_dir.exists():
        p.error(f'output directory already exists (will not overwrite): {a.output_dir}')
    return a


def engine_command(a, executable, onnx, engine, cache):
    # TensorRT 10.x options, including the 10.3 shipped with JetPack R36.4.7.
    return [str(executable), f'--onnx={onnx}', f'--saveEngine={engine}', '--fp16',
            f'--memPoolSize=workspace:{a.workspace_mib}', f'--builderOptimizationLevel={a.build_level}',
            f'--timingCacheFile={cache}', '--skipInference']


def run_logged(command, log):
    print(shlex.join(command), flush=True)
    with log.open('w') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=False)
    if result.returncode:
        raise RuntimeError(f'command exited {result.returncode}; see {log}')


def export_and_verify(a, onnx_path):
    # Lazy imports allow --help and --dry-run even on a host without ML packages.
    import cv2
    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch
    from torch import nn
    package = a.repo_root / 'lane_fusion_ws/src/autorace_lane_fusion_deep'
    sys.path.insert(0, str(package))
    from autorace_lane_fusion_deep.constants import CLASSES, MEAN, STD
    from autorace_lane_fusion_deep.runtime import blob, decode
    checkpoint = torch.load(a.weights, map_location='cpu', weights_only=False)
    if not isinstance(checkpoint, dict):
        raise ValueError('expected a state_dict or checkpoint dictionary')
    if 'classes' in checkpoint and tuple(checkpoint['classes']) != CLASSES:
        raise ValueError(f'checkpoint classes must be {CLASSES}; class order cannot be guessed')
    if a.factory:
        module, name = a.factory.rsplit(':', 1)
        model = getattr(importlib.import_module(module), name)()
    else:
        from autorace_lane_fusion_deep.model import LaneUNet
        model = LaneUNet(pretrained=False)
    state = checkpoint.get('model', checkpoint.get('state_dict', checkpoint.get('model_state_dict', checkpoint)))
    if not isinstance(state, dict):
        raise ValueError('model weights must be a state_dict')
    state = {key.removeprefix('module.'): value for key, value in state.items()}
    try:
        model.load_state_dict(state, strict=True)
    except RuntimeError as exc:
        raise ValueError('checkpoint architecture differs from LaneUNet/factory; specify the original model with --factory') from exc
    model = model.float().cpu().eval()
    size = (a.width, a.height) if a.width else checkpoint.get('size', (640, 480))
    if len(size) != 2 or any(int(v) <= 0 for v in size):
        raise ValueError('checkpoint size must be [width,height]')
    width, height = map(int, size)

    class MaskModel(nn.Module):
        def __init__(self):
            super().__init__()
            self.model = model
            self.register_buffer('scale', 1 / (255 * torch.tensor(STD).view(1, 3, 1, 1)))
            self.register_buffer('shift', (torch.tensor(MEAN) / torch.tensor(STD)).view(1, 3, 1, 1))

        def forward(self, image):
            logits = self.model(image * self.scale - self.shift)
            if isinstance(logits, dict):
                logits = logits['out']
            prob, label = torch.softmax(logits.float(), dim=1).max(dim=1)
            return label.to(torch.int32), prob

    head = MaskModel().eval()
    with torch.inference_mode():
        test_logits = model(torch.zeros(1, 3, height, width))
        if isinstance(test_logits, dict):
            test_logits = test_logits['out']
        if tuple(test_logits.shape) != (1, 3, height, width):
            raise ValueError('model must output three-class logits at the input resolution')
        export_options = dict(opset_version=17, input_names=['image'], output_names=['label', 'prob'],
                              do_constant_folding=True)
        # Keep the established static exporter on both older JetPack and newer PyTorch.
        import inspect
        if 'dynamo' in inspect.signature(torch.onnx.export).parameters:
            export_options['dynamo'] = False
        torch.onnx.export(head, torch.zeros(1, 3, height, width), str(onnx_path), **export_options)
    onnx.checker.check_model(onnx.load(str(onnx_path)))
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    session = ort.InferenceSession(str(onnx_path), sess_options=options, providers=['CPUExecutionProvider'])
    if a.images:
        files = sorted(p for p in a.images.iterdir() if p.suffix.lower() in ('.png', '.jpg', '.jpeg'))[:a.samples]
        if not files:
            raise ValueError('no validation images found')
        images = [cv2.imread(str(p)) for p in files]
        if any(image is None for image in images):
            raise ValueError('could not decode a validation image')
    else:
        rng = np.random.default_rng(0)
        images = [np.zeros((height, width, 3), np.uint8),
                  rng.integers(0, 256, (height, width, 3), dtype=np.uint8)]
    references, disagreement = [], []
    for image in images:
        x = blob(image, width, height)
        with torch.inference_mode():
            logits = model((torch.from_numpy(x) / 255 - torch.tensor(MEAN).view(1, 3, 1, 1)) /
                           torch.tensor(STD).view(1, 3, 1, 1))
            if isinstance(logits, dict):
                logits = logits['out']
            reference = decode(logits.detach().cpu().numpy(), (width, height), a.threshold)
        label, probability = session.run(['label', 'prob'], {'image': x})
        if not np.isfinite(probability).all():
            raise ValueError('nonfinite ONNX probability output')
        mask = label[0].astype(np.uint8)
        mask[probability[0] < a.threshold] = 0
        disagreement.append(float((mask != reference).mean()))
        references.append(reference)
    worst = max(disagreement)
    if worst > a.max_disagreement:
        raise ValueError(f'ONNX mask disagreement {worst:.6f} exceeds {a.max_disagreement}')
    return {'width': width, 'height': height, 'validation_images': len(images),
            'representative_images': a.images is not None, 'onnx_max_mask_disagreement': worst}, images, references


def main(argv=None):
    a = arguments(argv)
    onnx_path, engine = a.output_dir / 'lane_mask.onnx', a.output_dir / 'lane_mask.engine'
    executable = a.trtexec or shutil.which('trtexec') or '/usr/src/tensorrt/bin/trtexec'
    build = engine_command(a, executable, onnx_path, engine.with_suffix('.engine.building'), a.output_dir / 'timing.cache')
    if a.dry_run:
        print(f'Checkpoint: {a.weights}\nOutput: {a.output_dir}\nExport: static ONNX, RGB image -> label/prob')
        if not a.export_only:
            print(shlex.join(build))
        print('No files created; no models loaded; no engine built.')
        return 0
    if not a.export_only and not (Path(executable).is_file() and os.access(executable, os.X_OK)):
        raise SystemExit('trtexec not found. Run on the target Jetson or select --export-only.')
    a.output_dir.mkdir(parents=True, exist_ok=False)
    report = {'checkpoint': str(a.weights), 'fp16': not a.export_only, 'deployed': False,
              'status': 'started', 'threshold': a.threshold, 'build_command': build,
              'note': 'Synthetic checks are smoke tests; engine throughput is not ROS pipeline FPS.'}
    try:
        metadata, images, reference = export_and_verify(a, onnx_path)
        report.update(metadata)
        report['status'] = 'onnx_verified'
        if not a.export_only:
            run_logged(build, a.output_dir / 'build.log')
            temporary = engine.with_suffix('.engine.building')
            if not temporary.is_file() or temporary.stat().st_size == 0:
                raise RuntimeError('TensorRT did not create an engine')
            temporary.rename(engine)
            run_logged([str(executable), f'--loadEngine={engine}', '--warmUp=1000',
                        f'--duration={a.benchmark_seconds}', '--iterations=50',
                        f'--exportTimes={a.output_dir / "timings.json"}'], a.output_dir / 'benchmark.log')
            report['engine_mask_verified'] = False
            if a.check_engine:
                import numpy as np
                from autorace_lane_fusion_deep.runtime import TensorRTBackend, segment
                backend = TensorRTBackend(engine, metadata['width'], metadata['height'])
                try:
                    worst = max(float((segment(backend, image, metadata['width'], metadata['height'], a.threshold) != mask).mean())
                                for image, mask in zip(images, reference))
                finally:
                    backend.close()
                report['engine_max_mask_disagreement'] = worst
                if worst > a.max_disagreement:
                    raise ValueError(f'FP16 engine mask disagreement {worst:.6f} exceeds {a.max_disagreement}')
                report['engine_mask_verified'] = True
            report['status'] = 'engine_built'
        print(f'Finished: {a.output_dir}\nExisting model and ROS configuration were not changed.')
    except Exception as exc:
        report['status'], report['error'] = 'failed', str(exc)
        raise
    finally:
        (a.output_dir / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ImportError, ValueError, RuntimeError, OSError) as error:
        sys.exit(f'Optimization failed: {error}')
