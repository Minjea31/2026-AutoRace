#!/usr/bin/env python3
"""YOLO .pt -> FP32 ONNX -> TensorRT FP16, with FP32 input/output.

실행 예: python optimize_model.py best.pt
기본 입력: ../model/best.pt
기본 출력: ../model/best_fp16.engine
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[1]
VENDOR = WORKSPACE / "yolo26"


def log(message):
    print(message, flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("model", nargs="?", default="best.pt",
                        help="../model 안의 .pt 이름 또는 .pt 전체 경로")
    parser.add_argument("--output", type=Path, help="출력 .engine 경로; 기본: ../model/<이름>_fp16.engine")
    parser.add_argument("--imgsz", type=int, nargs="+", default=[640],
                        help="입력 크기: 640 또는 높이 너비. 현재 ROS 노드는 640 640")
    parser.add_argument("--device", type=int, default=0, help="CUDA GPU 번호 (기본 0)")
    parser.add_argument("--workspace-mib", type=int, default=512, help="TensorRT workspace 상한 MiB")
    parser.add_argument("--onnx-only", action="store_true", help="FP32 ONNX까지만 생성")
    parser.add_argument("--dry-run", action="store_true", help="경로와 설정만 출력; 모델 변환 안 함")
    parser.add_argument("--overwrite", action="store_true",
                        help="성공한 결과로 기존 출력 교체; 기존 파일은 .bak_시간으로 보존")
    args = parser.parse_args(argv)
    if len(args.imgsz) not in (1, 2) or any(v <= 0 or v % 32 for v in args.imgsz):
        parser.error("--imgsz는 32의 배수 1개 또는 높이 너비 2개여야 합니다.")
    if args.device < 0 or args.workspace_mib <= 0:
        parser.error("--device는 0 이상, --workspace-mib는 양수여야 합니다.")
    return args


def resolve_paths(args):
    supplied = Path(args.model).expanduser()
    candidates = ([supplied] if supplied.is_absolute() else
                  [WORKSPACE / "model" / supplied, Path.cwd() / supplied, WORKSPACE / supplied])
    source = next((p.resolve() for p in candidates if p.is_file()), None)
    if source is None or source.suffix.lower() != ".pt":
        raise ValueError("입력 .pt를 찾을 수 없습니다: " + args.model)
    output = (args.output.expanduser().resolve() if args.output else
              source.with_name(source.stem + "_fp16.engine"))
    if output.suffix.lower() != ".engine":
        raise ValueError("--output 확장자는 .engine이어야 합니다.")
    onnx = output.with_suffix(".onnx")
    report = output.with_suffix(".optimization.json")
    protected = {source, source.with_suffix(".onnx"), source.with_suffix(".engine")}
    if any(p in protected for p in (output, onnx, report)):
        raise ValueError("원본과 같은 이름의 산출물은 보존합니다. --output에 다른 이름을 지정하세요.")
    targets = [onnx, report] + ([] if args.onnx_only else [output])
    occupied = [str(p) for p in targets if p.exists()]
    if occupied and not args.overwrite and not args.dry_run:
        raise FileExistsError("출력이 이미 있습니다. 새 --output 또는 --overwrite를 사용하세요: "
                              + ", ".join(occupied))
    return source, output, onnx, report


def sha256(path):
    result = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def publish_file(staged, destination, overwrite):
    """Commit a completed artifact, without overwriting an existing file silently."""
    if destination.exists():
        if not overwrite:
            raise FileExistsError("출력이 이미 있습니다: " + str(destination))
        backup = destination.with_name(destination.name + ".bak_" +
                                       datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        os.replace(destination, backup)
        try:
            os.replace(staged, destination)
        except BaseException:
            os.replace(backup, destination)
            raise
        log("기존 결과 보관: " + str(backup))
    else:
        # Link creation fails atomically if another process has just created the target.
        os.link(staged, destination)
        staged.unlink()


def export_onnx(source, output, height, width, device):
    # Use the checked-in YOLO26 code, not an older pip YOLO incapable of loading it.
    if not (VENDOR / "ultralytics").is_dir():
        raise RuntimeError("YOLO26 소스가 없습니다: " + str(VENDOR))
    sys.path.insert(0, str(VENDOR))
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    import torch
    import onnx
    import ultralytics
    from ultralytics import YOLO

    if not torch.cuda.is_available() or device >= torch.cuda.device_count():
        raise RuntimeError("CUDA GPU를 찾을 수 없습니다. conda activate dl_env 후 대상 Jetson에서 실행하세요.")
    torch.cuda.set_device(device)
    model = YOLO(str(source), task="detect")
    if model.task != "detect":
        raise ValueError("detect 모델만 지원합니다.")
    metadata = {}

    def choose_destination(exporter):
        # Exporter calls this after model preparation and before writing ONNX.
        # Redirect output without copying/changing the source .pt.
        exporter.file = output.with_suffix(".pt")
        metadata.update(copy.deepcopy(exporter.metadata))

    model.add_callback("on_export_start", choose_destination)
    log("모델: " + str(source))
    log("Ultralytics: " + ultralytics.__version__ + " (" + ultralytics.__file__ + ")")
    log("클래스: " + json.dumps(model.names, ensure_ascii=False))
    exported = Path(model.export(format="onnx", imgsz=(height, width), batch=1,
                                  dynamic=False, half=False, simplify=False,
                                  opset=17, device=device, nms=False))
    if exported.resolve() != output.resolve() or not output.is_file():
        raise RuntimeError("예상 ONNX 출력과 다릅니다: " + str(exported))
    graph = onnx.load(str(output))
    onnx.checker.check_model(graph)
    tensor = graph.graph.input[0]
    shape = tuple(d.dim_value for d in tensor.type.tensor_type.shape.dim)
    if len(graph.graph.input) != 1 or shape != (1, 3, height, width):
        raise RuntimeError("ONNX 고정 입력이 잘못되었습니다: " + str(shape))
    if tensor.type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
        raise RuntimeError("ONNX 입력은 FP32여야 합니다.")
    if any(t.type.tensor_type.elem_type != onnx.TensorProto.FLOAT for t in graph.graph.output):
        raise RuntimeError("ONNX 출력은 FP32여야 합니다.")
    environment = {"python": sys.version.split()[0], "torch": torch.__version__,
                   "ultralytics": ultralytics.__version__, "onnx": onnx.__version__,
                   "gpu": torch.cuda.get_device_name(device)}
    return metadata, environment


def build_engine(onnx_path, engine_path, metadata, shape, workspace_mib):
    import tensorrt as trt

    major = int(trt.__version__.split(".")[0])
    if major != 10:
        raise RuntimeError("이 스크립트는 현재 장비의 TensorRT 10.x용입니다. 감지 버전: " + trt.__version__)
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    if not parser.parse_from_file(str(onnx_path)):
        raise RuntimeError("TensorRT ONNX 분석 실패:\n" +
                           "\n".join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    if network.num_inputs != 1 or tuple(network.get_input(0).shape) != shape:
        raise RuntimeError("TensorRT 입력 크기가 예상과 다릅니다.")
    for tensor in ([network.get_input(0)] +
                   [network.get_output(i) for i in range(network.num_outputs)]):
        if tensor.dtype != trt.float32:
            raise RuntimeError("TensorRT 입출력은 FP32여야 합니다: " + tensor.name)
    if not builder.platform_has_fast_fp16:
        raise RuntimeError("현재 GPU는 고속 FP16 빌드를 지원하지 않습니다.")
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_mib * 1024 * 1024)
    config.set_flag(trt.BuilderFlag.FP16)
    log("TensorRT FP16 빌드 시작: 수 분 걸릴 수 있습니다.")
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT 엔진 빌드에 실패했습니다.")
    engine_bytes = bytes(serialized)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(engine_bytes)
    if engine is None:
        raise RuntimeError("빌드된 엔진을 다시 열 수 없습니다.")
    tensors = []
    for index in range(engine.num_io_tensors):
        name = engine.get_tensor_name(index)
        if engine.get_tensor_dtype(name) != trt.float32:
            raise RuntimeError("빌드된 엔진의 입출력이 FP32가 아닙니다: " + name)
        tensors.append({"name": name, "shape": list(engine.get_tensor_shape(name)),
                        "dtype": str(engine.get_tensor_dtype(name)),
                        "mode": str(engine.get_tensor_mode(name))})
    meta = copy.deepcopy(metadata)
    meta.setdefault("args", {}).update(half=True, int8=False, dynamic=False, nms=False)
    encoded = json.dumps(meta, ensure_ascii=True).encode("utf-8")
    # Ultralytics AutoBackend expects a 4-byte metadata length, JSON, then the plan.
    with engine_path.open("xb") as handle:
        handle.write(len(encoded).to_bytes(4, "little", signed=True))
        handle.write(encoded)
        handle.write(engine_bytes)
    return {"tensorrt": trt.__version__, "io_tensors": tensors}


def main(argv=None):
    args = parse_args(argv)
    source, engine_path, onnx_path, report_path = resolve_paths(args)
    height, width = args.imgsz if len(args.imgsz) == 2 else args.imgsz * 2
    shape = (1, 3, height, width)
    log("입력: " + str(source))
    log("출력: " + str(onnx_path) + ("" if args.onnx_only else " / " + str(engine_path)))
    log(f"설정: 고정 {shape}, batch=1, 내부 FP16 / 입출력 FP32, workspace={args.workspace_mib} MiB")
    if (height, width) != (640, 640):
        log("참고: 현재 ROS YOLO 노드는 640x640을 사용합니다. 다른 크기는 노드 입력도 맞춰야 합니다.")
    if args.dry_run:
        log("경로 확인 완료. 파일은 생성하지 않았습니다.")
        return 0
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    # All work files stay beside the requested output, never in /tmp.
    work = engine_path.parent / (".optimize_" + engine_path.stem + "_" +
                                  datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    work.mkdir()
    stage_onnx = work / onnx_path.name
    stage_engine = work / engine_path.name
    stage_report = work / report_path.name
    started = time.monotonic()
    original_hash = sha256(source)
    try:
        metadata, environment = export_onnx(source, stage_onnx, height, width, args.device)
        report = {"source": str(source), "source_sha256": original_hash, "shape": list(shape),
                  "workspace_mib": args.workspace_mib, "internal_precision": "fp16",
                  "io_precision": "fp32", "dynamic": False, "nms": False, "opset": 17,
                  "metadata": metadata, "environment": environment,
                  "onnx_only": args.onnx_only,
                  "references": ["/home/wego/LANE_FPS_OPTIMIZATION.md",
                                 "/home/wego/lane_fps_optimization.patch"]}
        if not args.onnx_only:
            report["engine"] = build_engine(stage_onnx, stage_engine, metadata, shape,
                                            args.workspace_mib)
        if sha256(source) != original_hash:
            raise RuntimeError("실행 중 원본 .pt가 변경되었습니다. 산출물을 반영하지 않습니다.")
        report["elapsed_sec"] = round(time.monotonic() - started, 3)
        report["created_at"] = datetime.now().astimezone().isoformat()
        stage_report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        publish_file(stage_onnx, onnx_path, args.overwrite)
        if not args.onnx_only:
            publish_file(stage_engine, engine_path, args.overwrite)
        publish_file(stage_report, report_path, args.overwrite)
        work.rmdir()
    except BaseException:
        log("중간 결과/오류 확인 경로: " + str(work))
        raise
    log("완료: " + str(onnx_path))
    if not args.onnx_only:
        log("완료: " + str(engine_path))
        log("ROS 적용 시 model_path를 위 .engine 경로로 지정하세요.")
    log("기록: " + str(report_path))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        log("오류: " + str(error))
        log("실행 환경: conda activate dl_env")
        raise SystemExit(1)
