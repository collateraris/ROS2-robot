"""Compare Python's ioctl layout with real installed Linux UAPI declarations."""
import ctypes
import json
from pathlib import Path
import shutil
import subprocess

import pytest

from room_world_model import camera_v4l2 as v


def test_linux_ioctl_numbers_sizes_and_buffer_offsets_match_kernel_headers(tmp_path):
    cc = shutil.which("cc") or shutil.which("gcc")
    if cc is None or not Path("/usr/include/linux/videodev2.h").exists():
        pytest.skip("ABI verification needs a C compiler and Linux UAPI headers")
    mappings = {"v4l2_capability": v.Capability, "v4l2_pix_format": v.PixFormat,
                "v4l2_format": v.Format, "v4l2_requestbuffers": v.RequestBuffers,
                "v4l2_buffer": v.Buffer, "v4l2_captureparm": v.CaptureParm, "v4l2_streamparm": v.StreamParm,
                "v4l2_control": v.Control}
    ioctl_names = ["VIDIOC_QUERYCAP", "VIDIOC_G_FMT", "VIDIOC_S_FMT", "VIDIOC_REQBUFS",
                   "VIDIOC_QUERYBUF", "VIDIOC_QBUF", "VIDIOC_DQBUF", "VIDIOC_STREAMON",
                   "VIDIOC_STREAMOFF", "VIDIOC_G_PARM", "VIDIOC_S_PARM", "VIDIOC_G_CTRL", "VIDIOC_S_CTRL"]
    offsets = ["index", "timestamp", "timecode", "sequence", "memory", "m", "length", "request_fd"]
    source = '#include <stdio.h>\n#include <stddef.h>\n#include <linux/videodev2.h>\nint main(void) {\n'
    for name in mappings:
        source += f'printf("{name} %zu\\n", sizeof(struct {name}));\n'
    for name in ioctl_names:
        source += f'printf("{name} %lu\\n", (unsigned long){name});\n'
    for name in offsets:
        source += f'printf("offset_{name} %zu\\n", offsetof(struct v4l2_buffer, {name}));\n'
    source += 'return 0; }\n'
    path, binary = tmp_path / "abi.c", tmp_path / "abi"
    path.write_text(source)
    subprocess.run([cc, "-Wall", "-Werror", str(path), "-o", str(binary)], check=True, capture_output=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    observed = {name: int(size) for name, size in (line.split() for line in result.stdout.splitlines())}
    for name, cls in mappings.items():
        assert ctypes.sizeof(cls) == observed[name], name
    for name in ioctl_names:
        assert getattr(v, name) == observed[name], name
    for name in offsets:
        assert getattr(v.Buffer, name).offset == observed["offset_" + name], name


def test_yuyv_decoder_respects_driver_stride_padding():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    camera = v.V4L2Camera.__new__(v.V4L2Camera)
    camera.width = camera.height = 16
    camera.bytesperline = 40
    camera.pixel_format = "YUYV"
    rows = np.zeros((16, 40), dtype=np.uint8)
    rows[:, :32] = [100, 128, 100, 128] * 8
    rows[:, 32:] = 255
    raw = v.RawFrame(rows.tobytes(), 1, 0, 0, 1)
    decoded = camera.decode(raw)
    assert decoded.shape == (16, 16, 3)
    np.testing.assert_array_equal(decoded, np.broadcast_to(decoded[0, 0], decoded.shape))
