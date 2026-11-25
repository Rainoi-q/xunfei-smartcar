"""OCR 的纯逻辑：图像预处理、tesseract 调用、TSV 解析与文本清洗。

车上已离线安装 tesseract 4.1.1 + chi_sim/eng/osd，因此这里直接调用
命令行可执行文件，不依赖 pytesseract / PaddleOCR，省内存也省安装。
"""

import re
import shutil
import subprocess

import numpy as np


def tesseract_available():
    return shutil.which('tesseract') is not None


def preprocess_for_ocr(frame_bgr, roi=(0.0, 0.0, 1.0, 1.0), scale=1.5,
                       method='otsu', clahe=True):
    """裁剪 ROI、放大、增强，返回适合 OCR 的灰度图。

    method: otsu | adaptive | none
    """
    import cv2

    height, width = frame_bgr.shape[:2]
    x0 = int(max(0.0, min(1.0, roi[0])) * width)
    y0 = int(max(0.0, min(1.0, roi[1])) * height)
    x1 = int(max(0.0, min(1.0, roi[2])) * width)
    y1 = int(max(0.0, min(1.0, roi[3])) * height)
    if x1 <= x0 or y1 <= y0:
        x0, y0, x1, y1 = 0, 0, width, height

    crop = frame_bgr[y0:y1, x0:x1]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    if scale and abs(scale - 1.0) > 1e-3:
        gray = cv2.resize(gray, None, fx=float(scale), fy=float(scale),
                          interpolation=cv2.INTER_CUBIC)

    if clahe:
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)

    if method == 'otsu':
        _, gray = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    elif method == 'adaptive':
        gray = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 10)
    elif method == 'none':
        pass
    else:
        raise ValueError('未知预处理方式：%s' % method)

    return gray, (x0, y0, x1, y1)


def run_tesseract_tsv(image_path, lang='chi_sim+eng', psm=6, timeout=10.0):
    """调用 tesseract 输出 TSV，返回 (tsv_text, error)。"""
    cmd = ['tesseract', str(image_path), 'stdout',
           '-l', str(lang), '--psm', str(int(psm)), 'tsv']
    try:
        proc = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return '', str(exc)
    if proc.returncode != 0:
        return '', proc.stderr.decode('utf-8', errors='ignore')[:200]
    return proc.stdout.decode('utf-8', errors='ignore'), ''


_CONF_RE = re.compile(r'^-?\d+(\.\d+)?$')


def parse_tesseract_tsv(tsv_text, min_conf=30.0):
    """解析 TSV，得到按行组织的文本与平均置信度。

    返回 (text, mean_confidence, word_count)。
    """
    lines = {}
    confidences = []
    for row in (tsv_text or '').splitlines()[1:]:  # 首行是表头
        cols = row.split('\t')
        if len(cols) < 12:
            continue
        try:
            block, par, line, word = (int(cols[2]), int(cols[3]),
                                      int(cols[4]), int(cols[5]))
            conf = float(cols[10])
        except (ValueError, IndexError):
            continue
        text = cols[11].strip()
        if not text:
            continue
        if conf >= 0:
            confidences.append(conf)
        if conf < min_conf and conf >= 0:
            continue
        key = (block, par, line)
        lines.setdefault(key, []).append((word, text))

    ordered = []
    for key in sorted(lines.keys()):
        words = [text for _, text in sorted(lines[key], key=lambda item: item[0])]
        ordered.append(''.join(words) if _is_cjk_only(words) else ' '.join(words))

    text = clean_text('\n'.join(ordered))
    mean_conf = float(np.mean(confidences)) if confidences else 0.0
    return text, mean_conf, len(confidences)


def _is_cjk_only(words):
    if not words:
        return False
    for word in words:
        if not all(_is_cjk(ch) for ch in word):
            return False
    return True


def _is_cjk(ch):
    return '一' <= ch <= '鿿' or '㐀' <= ch <= '䶿'


def clean_text(text, max_chars=200):
    """去噪：合并空白、去掉 OCR 常见的孤立符号。"""
    if not text:
        return ''
    lines = []
    for raw in str(text).splitlines():
        line = raw.strip()
        line = re.sub(r'[ \t]+', ' ', line)
        line = re.sub(r'[\x00-\x1f]+', '', line)
        if not line:
            continue
        # 去掉纯符号行，例如 "|"、"~~"
        if not re.search(r'[0-9A-Za-z一-鿿]', line):
            continue
        lines.append(line)
    out = ' '.join(lines)
    return out[:max_chars].strip()


def text_quality(text):
    """有效字符占比，用于过滤乱码。"""
    if not text:
        return 0.0
    valid = sum(1 for ch in text
                if ch.isalnum() or _is_cjk(ch) or ch in '.,-_:%/')
    return valid / max(1, len(text))
