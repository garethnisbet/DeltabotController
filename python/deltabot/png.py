"""Minimal PNG reading and writing with only zlib and NumPy.

Enough for scan images: 8 or 16 bit, greyscale, grey + alpha, RGB or RGBA,
non-interlaced.  Written files use no row filtering; the reader undoes all
five filter types, so it also reads PNGs saved by other programs.
"""

import struct
import zlib
from pathlib import Path

import numpy as np

_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_COLOUR_TYPES = {1: 0, 2: 4, 3: 2, 4: 6}          # channels -> PNG colour type
_CHANNELS = {v: k for k, v in _COLOUR_TYPES.items()}


def _chunk(kind: bytes, data: bytes) -> bytes:
    return (struct.pack(">I", len(data)) + kind + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))


def write_png(path, image: np.ndarray, compression: int = 6) -> None:
    """Save a uint8 or uint16 array, shaped (h, w) or (h, w, 1-4), as PNG."""
    img = np.asarray(image)
    if img.dtype not in (np.uint8, np.uint16):
        raise TypeError(f"PNG needs uint8 or uint16 data, not {img.dtype}")
    if img.ndim == 2:
        img = img[:, :, None]
    if img.ndim != 3 or img.shape[2] not in _COLOUR_TYPES:
        raise ValueError(f"cannot store an image shaped {image.shape} as PNG")
    h, w, channels = img.shape
    depth = 8 * img.dtype.itemsize
    rows = img.astype(img.dtype.newbyteorder(">"), copy=False).reshape(h, -1)
    raw = np.hstack([np.zeros((h, 1), np.uint8), rows.view(np.uint8).reshape(h, -1)])
    header = struct.pack(">IIBBBBB", w, h, depth, _COLOUR_TYPES[channels], 0, 0, 0)
    Path(path).write_bytes(_SIGNATURE + _chunk(b"IHDR", header)
                           + _chunk(b"IDAT", zlib.compress(raw.tobytes(), compression))
                           + _chunk(b"IEND", b""))


def read_png(path) -> np.ndarray:
    """Load a non-interlaced 8 or 16 bit PNG as (h, w) or (h, w, channels)."""
    data = Path(path).read_bytes()
    if not data.startswith(_SIGNATURE):
        raise ValueError(f"{path} is not a PNG file")
    pos, idat, header = len(_SIGNATURE), [], None
    while pos < len(data):
        length, kind = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            idat.append(body)
        elif kind == b"IEND":
            break
    if header is None:
        raise ValueError(f"{path} has no IHDR chunk")
    w, h, depth, colour, _, _, interlace = header
    if depth not in (8, 16) or colour not in _CHANNELS or interlace:
        raise ValueError(f"{path}: only non-interlaced 8/16 bit grey, grey+alpha, "
                         f"RGB or RGBA PNGs are supported")
    channels = _CHANNELS[colour]
    bpp = channels * depth // 8                      # bytes per pixel
    stride = w * bpp
    raw = np.frombuffer(zlib.decompress(b"".join(idat)), np.uint8).reshape(h, stride + 1)
    out = np.zeros((h, stride), np.uint8)
    prev = np.zeros(stride, np.int32)
    for y in range(h):
        kind, line = raw[y, 0], raw[y, 1:].astype(np.int32)
        if kind == 0:
            cur = line
        elif kind == 2:
            cur = (line + prev) & 0xFF
        elif kind in (1, 3, 4):
            cur = line.copy()
            for x in range(stride):                  # needs the pixel to its left
                left = cur[x - bpp] if x >= bpp else 0
                if kind == 1:
                    pred = left
                elif kind == 3:
                    pred = (left + prev[x]) >> 1
                else:
                    up_left = prev[x - bpp] if x >= bpp else 0
                    p = left + prev[x] - up_left
                    pa, pb, pc = abs(p - left), abs(p - prev[x]), abs(p - up_left)
                    pred = left if pa <= pb and pa <= pc else prev[x] if pb <= pc else up_left
                cur[x] = (cur[x] + pred) & 0xFF
        else:
            raise ValueError(f"{path}: bad PNG filter type {kind}")
        out[y] = cur
        prev = cur
    dtype = np.uint8 if depth == 8 else np.dtype(">u2")
    img = out.view(dtype).reshape(h, w, channels).astype(
        np.uint8 if depth == 8 else np.uint16)
    return img[:, :, 0] if channels == 1 else img
