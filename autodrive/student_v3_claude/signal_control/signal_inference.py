"""Optional ONNX semantic-mask adapter for the signal classifier."""
import json
import math
import inspect
import os
import select
import struct
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np


def letterbox(image, width, height, value=0):
    source_height, source_width = image.shape[:2]
    scale = min(float(width) / source_width, float(height) / source_height)
    resized_width = max(1, round(source_width * scale))
    resized_height = max(1, round(source_height * scale))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    left, top = (width - resized_width) // 2, (height - resized_height) // 2
    canvas = np.full((height, width, image.shape[2]), value, dtype=image.dtype)
    canvas[top:top + resized_height, left:left + resized_width] = resized
    return canvas, {"left": left, "top": top, "width": resized_width, "height": resized_height}


class SemanticSignal:
    """Convert BGR camera frames to the trained model's traffic-light mask."""

    def __init__(self, model_path, require_gpu=False):
        import onnxruntime as ort

        model = Path(model_path)
        metadata = json.loads(model.with_suffix(model.suffix + ".json").read_text(encoding="utf-8"))
        self.width, self.height = int(metadata["input_width"]), int(metadata["input_height"])
        self.class_id = metadata["class_names"].index("traffic_light")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        options.add_session_config_entry("session.inter_op.allow_spinning", "0")
        providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider")
                     if p in ort.get_available_providers()]
        if "CUDAExecutionProvider" in providers and hasattr(ort, "preload_dlls"):
            ort.preload_dlls()
        self.session = ort.InferenceSession(str(model), sess_options=options, providers=providers)
        if require_gpu and self.session.get_providers()[0] != "CUDAExecutionProvider":
            raise RuntimeError("CUDA signal inference requested but unavailable")
        self.input_name = self.session.get_inputs()[0].name
        if "logits" not in [output.name for output in self.session.get_outputs()]:
            raise RuntimeError("signal model must expose logits output")

    def mask(self, image_bgr, threshold):
        if not math.isfinite(threshold) or not 0.05 <= threshold <= 0.95:
            raise ValueError("threshold must be 0.05..0.95")
        canvas, pad = letterbox(image_bgr, self.width, self.height)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        rgb = (rgb - np.asarray([0.485, 0.456, 0.406], np.float32)) / np.asarray(
            [0.229, 0.224, 0.225], np.float32)
        tensor = rgb.transpose(2, 0, 1)[None]
        logits = self.session.run(["logits"], {self.input_name: tensor})[0][0, self.class_id]
        binary = (logits >= math.log(threshold / (1.0 - threshold))).astype(np.uint8) * 255
        binary = binary[pad["top"]:pad["top"] + pad["height"],
                        pad["left"]:pad["left"] + pad["width"]]
        return cv2.resize(binary, (image_bgr.shape[1], image_bgr.shape[0]),
                          interpolation=cv2.INTER_NEAREST)


class ContainerSemanticSignal:
    """Use the existing semantic GPU container when the host has no ONNX runtime.

Keep one inference process and exchange raw images over pipes. This neither
installs another runtime on the host nor changes the obstacle sidecar's model,
class selection, shared masks or UDP sockets.
    """
    def __init__(self, model_path):
        semantic_root = Path(__file__).resolve().parents[3] / "semantic"
        model = Path(model_path).resolve()
        try:
            mounted_model = Path("/competition/semantic") / model.relative_to(semantic_root)
        except ValueError:
            raise ValueError("container signal model must be under " + str(semantic_root)) from None
        script = ("import json,math,sys,struct\nfrom pathlib import Path\n"
                  "import cv2,numpy as np\nimport torch\n"
                  "cv2.setNumThreads(1)\ntorch.set_num_threads(1)\n" +
                  inspect.getsource(letterbox) + "\n" + inspect.getsource(SemanticSignal) + '''
def read_exact(n):
    out = bytearray()
    while len(out) < n:
        part = sys.stdin.buffer.read(n-len(out))
        if not part:
            raise EOFError()
        out.extend(part)
    return bytes(out)
model = SemanticSignal(sys.argv[1], require_gpu=True)
# CUDA's first graph execution can take seconds. Complete it before allowing
# the launcher to start driving or accepting a timestamped camera frame.
model.mask(np.zeros((model.height,model.width,3),np.uint8),.5)
sys.stdout.buffer.write(b'READY\\n'); sys.stdout.buffer.flush()
while True:
    try:
        width,height,threshold = struct.unpack('!IId',read_exact(16))
    except EOFError:
        break
    if not 1 <= width <= 4096 or not 1 <= height <= 4096:
        raise ValueError('invalid signal image dimensions')
    image = np.frombuffer(read_exact(width*height*3),np.uint8).reshape(height,width,3)
    mask = model.mask(image,threshold)
    sys.stdout.buffer.write(mask.tobytes()); sys.stdout.buffer.flush()
''')
        self.process = subprocess.Popen(
            ["docker", "exec", "-i", "alpamayo_semantic", "python", "-u", "-c", script, str(mounted_model)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.failed, self.closed = False, False
        try:
            if self._read_exact(6, 30.) != b"READY\n":
                raise RuntimeError("signal GPU worker startup handshake failed")
            os.set_blocking(self.process.stdin.fileno(), False)
        except Exception:
            self.close()
            raise

    def _read_exact(self, count, timeout=3.):
        deadline, output = time.monotonic()+timeout, bytearray()
        while len(output) < count:
            remaining = deadline-time.monotonic()
            if remaining <= 0. or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise TimeoutError("signal GPU worker timed out")
            part = os.read(self.process.stdout.fileno(), count-len(output))
            if not part:
                raise RuntimeError("signal GPU worker ended")
            output.extend(part)
        return bytes(output)

    def mask(self, image_bgr, threshold):
        height, width = image_bgr.shape[:2]
        if self.failed or self.closed or self.process.poll() is not None:
            raise RuntimeError("signal GPU worker is not running")
        try:
            self._write_exact(struct.pack("!IId", width, height, threshold))
            self._write_exact(memoryview(np.ascontiguousarray(image_bgr)).cast("B"))
            return np.frombuffer(self._read_exact(width*height), np.uint8).reshape(height, width)
        except Exception:
            # A partial/late response cannot be reused for a later frame.
            self.failed = True
            raise

    def _write_exact(self, data):
        remaining_data = memoryview(data)
        deadline, fd = time.monotonic()+3., self.process.stdin.fileno()
        while remaining_data:
            remaining = deadline-time.monotonic()
            if remaining <= 0. or not select.select([], [fd], [], remaining)[1]:
                raise TimeoutError("signal GPU input timed out")
            try:
                written = os.write(fd, remaining_data[:65536])
            except BlockingIOError:
                continue
            if not written:
                raise RuntimeError("signal GPU input pipe closed")
            remaining_data = remaining_data[written:]

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.process.stdin.close()
        try:
            self.process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.)
        self.process.stdout.close()


def make_signal_model(model_path):
    try:
        import onnxruntime  # noqa: F401
    except ImportError:
        return ContainerSemanticSignal(model_path)
    return SemanticSignal(model_path)
