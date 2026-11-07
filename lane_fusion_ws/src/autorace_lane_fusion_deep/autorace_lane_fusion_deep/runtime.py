"""Lazy backend imports: OpenCV and TensorRT paths do not load PyTorch."""
import ctypes
import ctypes.util
import glob
from pathlib import Path
import sys

import cv2
import numpy as np

from .constants import MEAN, STD


def preprocess(bgr, width, height):
    image = cv2.resize(bgr, (width, height), interpolation=cv2.INTER_LINEAR)
    output = np.empty((1, 3, height, width), dtype=np.float32)
    for channel in range(3):
        np.multiply(image[:, :, 2 - channel], np.float32(1 / (255 * STD[channel])),
                    out=output[0, channel], casting='unsafe')
        output[0, channel] -= np.float32(MEAN[channel] / STD[channel])
    return output


def blob(bgr, width, height):
    """Input of a MASK model: RGB 0..255 NCHW float32, resized in C++ (INTER_LINEAR).
    Normalisation, softmax and argmax run inside the model (on the GPU)."""
    return cv2.dnn.blobFromImage(bgr, 1.0, (width, height), swapRB=True)


def segment(backend, bgr, width, height, threshold=0.5):
    """Class mask (uint8 0/1/2, width x height) for either model interface:
    MASK (image -> label, prob: post-processing on the GPU) or LOGITS (rgb -> logits)."""
    if getattr(backend, 'interface', 'logits') == 'mask':
        out = backend.run(blob(bgr, width, height))
        mask = out['label'].reshape(height, width).astype(np.uint8)
        mask[out['prob'].reshape(height, width) < threshold] = 0
        return mask
    return decode(backend.logits(preprocess(bgr, width, height)), (width, height), threshold)


def decode(logits, original_size, threshold=0.5):
    if logits.ndim != 4 or logits.shape[0] != 1 or logits.shape[1] != 3 or not np.isfinite(logits).all():
        raise ValueError('Expected finite logits [1,3,H,W]')
    values = logits[0].astype(np.float32)
    maximum = cv2.max(cv2.max(values[0], values[1]), values[2])
    mask = np.zeros(maximum.shape, np.uint8)
    mask[values[1] > values[0]] = 1
    mask[(values[2] > values[0]) & (values[2] > values[1])] = 2
    denominator = np.zeros_like(maximum)
    for channel in values:
        denominator += np.exp(channel - maximum)
    mask[1.0 / denominator < threshold] = 0
    return cv2.resize(mask, original_size, interpolation=cv2.INTER_NEAREST)


class OpenCVBackend:
    name = 'opencv'

    def __init__(self, config):
        self.config = config

    def predict(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        orange = cv2.inRange(hsv, np.array(self.config['orange_lower']), np.array(self.config['orange_upper']))
        white = cv2.inRange(hsv, np.array(self.config['white_lower']), np.array(self.config['white_upper']))
        # Remove long connected solid-white marks. This is only a color/shape heuristic.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(white)
        for index in range(1, count):
            if stats[index, cv2.CC_STAT_HEIGHT] > bgr.shape[0] * self.config['white_max_height_ratio']:
                white[labels == index] = 0
        mask = np.zeros(bgr.shape[:2], np.uint8)
        mask[white > 0], mask[orange > 0] = 2, 1
        return mask


class TorchBackend:
    name = 'torch'

    def __init__(self, path, width, height, device='cuda'):
        import torch
        from .model import load_checkpoint
        self.torch, self.device = torch, torch.device(device)
        self.model, _ = load_checkpoint(path, self.device)
        self.model.eval()
        self.width, self.height = width, height

    def logits(self, rgb):
        with self.torch.inference_mode():
            return self.model(self.torch.from_numpy(rgb).to(self.device)).float().cpu().numpy()


class ONNXBackend:
    name = 'onnx'

    def __init__(self, path, width, height, device='cpu'):
        import onnxruntime as ort
        if device not in ('cpu', 'cuda'):
            raise ValueError('ONNX device must be cpu/cuda')
        providers = ['CPUExecutionProvider']
        if device == 'cuda':
            if 'CUDAExecutionProvider' not in ort.get_available_providers():
                raise RuntimeError('CUDA ONNX Runtime provider is unavailable; select device=cpu explicitly')
            providers.insert(0, 'CUDAExecutionProvider')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=providers)
        shape = self.session.get_inputs()[0].shape
        if shape != [1, 3, height, width]:
            raise ValueError(f'ONNX shape {shape} differs from configured size')
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]
        self.interface = 'mask' if self.input_name == 'image' else 'logits'
        self.width, self.height = width, height

    def run(self, x):
        return dict(zip(self.output_names, self.session.run(self.output_names, {self.input_name: x})))

    def logits(self, rgb):
        return self.session.run(['logits'], {'rgb': rgb})[0]


class Cuda:
    def __init__(self):
        candidates = [ctypes.util.find_library('cudart'), 'libcudart.so']
        candidates += sorted(glob.glob('/usr/local/cuda*/lib64/libcudart.so*'))
        candidates += sorted(glob.glob('/usr/local/cuda*/targets/aarch64-linux/lib/libcudart.so*'))
        candidates += sorted(glob.glob(str(Path(sys.prefix) / 'lib/libcudart.so*')))
        candidates += sorted(glob.glob(str(Path(sys.prefix) / 'lib/python*/site-packages/nvidia/cuda_runtime/lib/libcudart.so*')))
        self.lib = None
        for path in filter(None, candidates):
            try:
                self.lib = ctypes.CDLL(path)
                break
            except OSError:
                pass
        if self.lib is None:
            raise RuntimeError('libcudart not found; install/source the JetPack CUDA runtime')
        definitions = {
            'cudaSetDevice': [ctypes.c_int],
            'cudaMalloc': [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t],
            'cudaFree': [ctypes.c_void_p],
            'cudaMemcpy': [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int],
            'cudaStreamCreate': [ctypes.POINTER(ctypes.c_void_p)],
            'cudaStreamSynchronize': [ctypes.c_void_p],
            'cudaStreamDestroy': [ctypes.c_void_p],
        }
        for name, argtypes in definitions.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = argtypes, ctypes.c_int
        self.call('cudaSetDevice', 0)

    def call(self, name, *args):
        code = getattr(self.lib, name)(*args)
        if code:
            raise RuntimeError(f'{name} failed with CUDA error {code}')


class TensorRTBackend:
    """Static batch-1, explicit-batch TensorRT 8.5+ / 10.x engine runner."""
    name = 'tensorrt'

    def __init__(self, path, width, height, device='cuda'):
        import tensorrt as trt
        self.cuda = Cuda()
        self.allocations, self.stream = [], ctypes.c_void_p()
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(path).read_bytes())
        if self.engine is None:
            raise RuntimeError('Engine deserialization failed; rebuild on this Jetson/TensorRT version')
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError('Could not create TensorRT execution context')
        # Avoid old trt.nptype's dependency on removed NumPy aliases (np.bool).
        def numpy_dtype(value):
            if value == trt.DataType.FLOAT:
                return np.float32
            if value == trt.DataType.HALF:
                return np.float16
            if value == trt.DataType.INT32:
                return np.int32
            raise ValueError('Lane engine I/O must use float32/float16/int32')
        self.modern = hasattr(self.engine, 'num_io_tensors') and hasattr(self.context, 'execute_async_v3')
        if self.modern:
            names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
            shape = lambda name: tuple(self.engine.get_tensor_shape(name))
            dtype = lambda name: numpy_dtype(self.engine.get_tensor_dtype(name))
        else:
            names = [self.engine.get_binding_name(i) for i in range(self.engine.num_bindings)]
            shape = lambda name: tuple(self.engine.get_binding_shape(self.engine.get_binding_index(name)))
            dtype = lambda name: numpy_dtype(self.engine.get_binding_dtype(self.engine.get_binding_index(name)))
        # LOGITS interface: rgb -> logits [1,3,H,W]. MASK interface: image (RGB 0..255)
        # -> label int32 [1,H,W] + prob [1,H,W] (normalise/softmax/argmax on the GPU).
        if set(names) == {'rgb', 'logits'} and shape('logits') == (1, 3, height, width):
            self.interface, self.input_name, self.output_names = 'logits', 'rgb', ['logits']
        elif set(names) == {'image', 'label', 'prob'} and shape('label') == shape('prob') == (1, height, width):
            self.interface, self.input_name, self.output_names = 'mask', 'image', ['label', 'prob']
        else:
            raise ValueError('Engine must be rgb->logits or image->label/prob at the configured resolution')
        if shape(self.input_name) != (1, 3, height, width):
            raise ValueError('Engine input does not match configured resolution')
        self.input = np.empty(shape(self.input_name), dtype=dtype(self.input_name))
        self.outputs = {n: np.empty(shape(n), dtype=dtype(n)) for n in self.output_names}
        self.output = self.outputs.get('logits')
        self.cuda.call('cudaStreamCreate', ctypes.byref(self.stream))
        self.pointers = {}
        for name, array in ((self.input_name, self.input), *self.outputs.items()):
            pointer = ctypes.c_void_p()
            self.cuda.call('cudaMalloc', ctypes.byref(pointer), array.nbytes)
            self.allocations.append(pointer)
            self.pointers[name] = pointer
            if self.modern and not self.context.set_tensor_address(name, pointer.value):
                raise RuntimeError(f'TensorRT could not bind {name}')
        self.bindings = [self.pointers[name].value for name in names]
        self.width, self.height = width, height

    def run(self, x):
        np.copyto(self.input, x, casting='same_kind')
        self.cuda.call('cudaMemcpy', self.pointers[self.input_name], self.input.ctypes.data, self.input.nbytes, 1)
        if self.modern:
            ok = self.context.execute_async_v3(stream_handle=self.stream.value)
        else:
            ok = self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.value)
        if not ok:
            raise RuntimeError('TensorRT execution failed')
        self.cuda.call('cudaStreamSynchronize', self.stream)
        for name, array in self.outputs.items():
            self.cuda.call('cudaMemcpy', array.ctypes.data, self.pointers[name], array.nbytes, 2)
        return self.outputs

    def logits(self, rgb):
        return self.run(rgb)['logits'].astype(np.float32, copy=True)

    def close(self):
        if getattr(self, 'stream', None) and self.stream.value:
            self.cuda.call('cudaStreamSynchronize', self.stream)
        for pointer in getattr(self, 'allocations', []):
            self.cuda.call('cudaFree', pointer)
        self.allocations = []
        if getattr(self, 'stream', None) and self.stream.value:
            self.cuda.call('cudaStreamDestroy', self.stream)
            self.stream = ctypes.c_void_p()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass  # Interpreter shutdown/partial construction must not mask the original error.


def create_backend(kind, path, width, height, device='cuda'):
    if kind == 'auto':
        kind = {'.pt': 'torch', '.pth': 'torch', '.onnx': 'onnx', '.engine': 'tensorrt', '.plan': 'tensorrt'}.get(Path(path).suffix)
    choices = {'torch': TorchBackend, 'onnx': ONNXBackend, 'tensorrt': TensorRTBackend}
    if kind not in choices:
        raise ValueError('Choose torch/onnx/tensorrt or a recognized model suffix')
    return choices[kind](path, width, height, device)
