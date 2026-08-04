#!/usr/bin/env python3
"""sensor_msgs/Image -> numpy dönüşümü, tek yerde.

Hem kare yakalayan araç (grab_frames.py) hem uçuş sırasında çalışan tarama
düğümü (scripts/scan_boxes.py) aynı dönüşümü yapıyor. Kopyalanması riskli:
buradaki bir hata (yanlış kanal sırası, step'in width'ten büyük olduğu
durumun atlanması) kodların sessizce çözülememesi olarak görünür, çökme
olarak değil.

cv_bridge kullanılmıyor -- ek bir ROS paketi bağımlılığı getiriyor ve
yaptığı iş bu kadar.
"""

from __future__ import annotations

import numpy as np


def to_array(msg) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 RGB."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        # step >= width*3 olabilir (satır dolgusu); önce step'e göre şekillendirip
        # sonra kırpmak gerekiyor, doğrudan width'e reshape etmek kaymaya yol açar.
        arr = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        return arr[:, :, ::-1] if msg.encoding == "bgr8" else arr
    if msg.encoding == "mono8":
        return np.repeat(buf.reshape(msg.height, msg.step)[:, :msg.width, None], 3, axis=2)
    raise ValueError(f"desteklenmeyen encoding: {msg.encoding}")


def to_gray(rgb: np.ndarray) -> np.ndarray:
    """HxWx3 RGB -> HxW uint8. zbar zaten griye çeviriyor; burada bir kez
    yapıp vermek 1080p'de birkaç ms kazandırıyor."""
    if rgb.ndim == 2:
        return rgb
    return (rgb[:, :, 0] * 0.299 + rgb[:, :, 1] * 0.587
            + rgb[:, :, 2] * 0.114).astype(np.uint8)
