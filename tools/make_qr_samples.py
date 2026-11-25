"""生成可打印的二维码样张（不依赖 qrcode 库，直接画 qr_asset 里的矩阵）。

输出：
    out/qr_sample_A03.png   纯二维码（A4 打印后贴在投递点）

用法：
    python tools/make_qr_samples.py
    python tools/make_qr_samples.py --size-mm 200 --out out

打印建议：
    - 200 mm × 200 mm，哑光纸，四角胶带压平；
    - 与相机尽量“正视”，斜视超过约 45° pyzbar 会解不出；
    - 四周留白（静区）不要被裁掉，脚本已自动留 4 个模块。
"""

import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'simulation', 'virtual_sim'))

from virtual_sim.qr_asset import QR_MATRIX, QR_PAYLOAD  # noqa: E402

QUIET = 4  # 静区宽度（模块数），规范要求 >= 4


def _font(size):
    for name in ('arial.ttf', 'segoeui.ttf', 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            continue
    return ImageFont.load_default()


def render(dpi=300, size_mm=200):
    modules = len(QR_MATRIX) + 2 * QUIET
    side_px = max(modules, int(round(size_mm / 25.4 * dpi)))
    module_px = side_px // modules
    total = module_px * modules

    image = Image.new('RGB', (total, total), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    for row, line in enumerate(QR_MATRIX):
        for col, value in enumerate(line):
            if value != '1':
                continue
            x0 = (col + QUIET) * module_px
            y0 = (row + QUIET) * module_px
            draw.rectangle([x0, y0, x0 + module_px - 1,
                            y0 + module_px - 1], fill=(0, 0, 0))
    return image, total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default=os.path.join(ROOT, 'out'))
    parser.add_argument('--size-mm', type=float, default=200.0)
    parser.add_argument('--dpi', type=int, default=300)
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    image, side = render(dpi=args.dpi, size_mm=args.size_mm)
    path = os.path.join(args.out, 'qr_sample_A03.png')
    image.save(path, dpi=(args.dpi, args.dpi))

    print('已生成 %s（%d×%d px，约 %.0f mm 见方）' % (
        path, side, side, args.size_mm))
    print('内容：%s' % QR_PAYLOAD)
    print('打印后请用车上相机验证：ros2 topic echo /perception/qr_payload')
    return 0


if __name__ == '__main__':
    sys.exit(main())
