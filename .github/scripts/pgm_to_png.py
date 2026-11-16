"""Convert a saved map (binary PGM, as map_saver writes it) to a grayscale PNG.

Usage: python3 pgm_to_png.py <in.pgm> <out.png> [--scale N]

--scale repeats each pixel N times so a small map stays readable in a browser.
Standard library only, so a workflow can run it without installing anything.
"""

import argparse
import struct
import zlib


def read_pgm(path):
    data = open(path, 'rb').read()
    fields, pos = [], 0
    while len(fields) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(data[start:pos])
    magic, width, height, maxval = fields[0], int(fields[1]), int(fields[2]), int(fields[3])
    if magic != b'P5' or maxval > 255:
        raise ValueError(f'{path}: only 8-bit binary PGM (P5) is supported')
    pixels = data[pos + 1:pos + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError(f'{path}: expected {width * height} pixels, found {len(pixels)}')
    if maxval != 255:
        pixels = bytes(p * 255 // maxval for p in pixels)
    return width, height, pixels


def write_png(path, width, height, pixels, scale=1):
    rows = []
    for y in range(height):
        row = pixels[y * width:(y + 1) * width]
        wide = bytes(p for p in row for _ in range(scale))
        rows.extend([b'\x00' + wide] * scale)

    def chunk(kind, body):
        return (struct.pack('>I', len(body)) + kind + body
                + struct.pack('>I', zlib.crc32(kind + body) & 0xffffffff))

    header = struct.pack('>IIBBBBB', width * scale, height * scale, 8, 0, 0, 0, 0)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', header))
        f.write(chunk(b'IDAT', zlib.compress(b''.join(rows), 9)))
        f.write(chunk(b'IEND', b''))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('pgm')
    parser.add_argument('png')
    parser.add_argument('--scale', type=int, default=1)
    args = parser.parse_args()
    write_png(args.png, *read_pgm(args.pgm), scale=args.scale)


if __name__ == '__main__':
    main()
