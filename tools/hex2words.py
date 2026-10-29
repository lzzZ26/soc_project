"""Convert objcopy -O verilog byte hex (image.hex) to word-oriented hex.

Reads space-separated byte tokens with @byte-address directives,
packs 4 consecutive bytes little-endian into 32-bit words, and writes
one 32-bit token per word starting at @00000000, zero-filling the full
depth so the file matches cmsdk_axi_ram_beh's word array exactly.

Usage: python hex2words.py <src.hex> <dst.hex> [nwords]
       default nwords = 16384 (64KB -> AW=16)
"""
import sys


def main():
    src, dst = sys.argv[1], sys.argv[2]
    nwords = int(sys.argv[3]) if len(sys.argv) > 3 else 16384

    mem = {}
    addr = 0
    for tok in open(src).read().split():
        if tok.startswith('@'):
            addr = int(tok[1:], 16)
        else:
            mem[addr] = int(tok, 16)
            addr += 1

    if mem and max(mem) >= nwords * 4:
        sys.exit("image too large: %d bytes >= %d" % (max(mem) + 1, nwords * 4))

    with open(dst, 'w') as f:
        f.write('@00000000\n')
        for w in range(nwords):
            b = w * 4
            word = (mem.get(b, 0)
                    | (mem.get(b + 1, 0) << 8)
                    | (mem.get(b + 2, 0) << 16)
                    | (mem.get(b + 3, 0) << 24))
            f.write('%08x%s' % (word, '\n' if (w % 8) == 7 else ' '))
        if nwords % 8:
            f.write('\n')
    print('hex2words: %s -> %s (%d words)' % (src, dst, nwords))


if __name__ == '__main__':
    main()
