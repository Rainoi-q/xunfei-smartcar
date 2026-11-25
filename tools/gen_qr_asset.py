"""重新生成仿真用的二维码位图（本地运行，不需要 ROS）。

只用 qrcode 库把固定订单 JSON 转成 0/1 矩阵，写回
    src/simulation/virtual_sim/virtual_sim/qr_asset.py
矩阵不带静区（边框由 World 贴图时留白）。

用法：
    python tools/gen_qr_asset.py
    python tools/gen_qr_asset.py --ecc M --preview out/qr_preview.png
"""

import argparse
import os
import sys

try:
    import qrcode
    from qrcode.constants import (ERROR_CORRECT_H, ERROR_CORRECT_L,
                                  ERROR_CORRECT_M, ERROR_CORRECT_Q)
except ImportError:
    sys.exit('缺少 qrcode：python -m pip install --user qrcode')

ECCS = {
    'L': ERROR_CORRECT_L,
    'M': ERROR_CORRECT_M,
    'Q': ERROR_CORRECT_Q,
    'H': ERROR_CORRECT_H,
}

PAYLOAD = ('{"order_id":"A03","consignee":"zhang_san",'
           '"destination":"door_3","priority":2,"weight_kg":1.5}')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = os.path.join(ROOT, 'src', 'simulation', 'virtual_sim',
                      'virtual_sim', 'qr_asset.py')


def build_matrix(payload, ecc='L'):
    qr = qrcode.QRCode(error_correction=ECCS[ecc], box_size=1, border=0)
    qr.add_data(payload)
    qr.make(fit=True)
    return qr.get_matrix(), qr.version


def write_asset(matrix, version, ecc):
    lines = [
        '"""仿真与演示用二维码位图（离线生成，避免车上再装 qrcode 库）。',
        '',
        '内容与 docs/接口文档.md 的二维码 JSON 约定一致；',
        '需要打印实物时运行 tools/make_qr_samples.py。',
        '由 tools/gen_qr_asset.py 生成，不要手改。',
        '"""',
        '',
        'QR_PAYLOAD = %r' % PAYLOAD,
        '',
        '# version %d, ECC %s, %d x %d 模块（不含静区）' % (
            version, ecc, len(matrix), len(matrix)),
        'QR_MATRIX = [',
    ]
    for row in matrix:
        lines.append('    %r,' % ''.join('1' if cell else '0' for cell in row))
    lines.append(']')
    lines.append('')
    with open(TARGET, 'w', encoding='utf-8', newline='\n') as handle:
        handle.write('\n'.join(lines))
    return TARGET


def write_preview(matrix, path):
    from PIL import Image
    scale = 8
    border = 4
    n = len(matrix)
    side = (n + border * 2) * scale
    image = Image.new('L', (side, side), 255)
    pixels = image.load()
    for row, line in enumerate(matrix):
        for col, cell in enumerate(line):
            if not cell:
                continue
            x0 = (col + border) * scale
            y0 = (row + border) * scale
            for y in range(y0, y0 + scale):
                for x in range(x0, x0 + scale):
                    pixels[x, y] = 0
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    image.save(path)
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ecc', default='L', choices=sorted(ECCS))
    parser.add_argument('--payload', default=PAYLOAD)
    parser.add_argument('--preview', default='')
    args = parser.parse_args()

    matrix, version = build_matrix(args.payload, args.ecc)
    print('version %d，%d x %d 模块' % (version, len(matrix), len(matrix)))
    print('写入', write_asset(matrix, version, args.ecc))
    if args.preview:
        print('预览', write_preview(matrix, args.preview))


if __name__ == '__main__':
    main()
