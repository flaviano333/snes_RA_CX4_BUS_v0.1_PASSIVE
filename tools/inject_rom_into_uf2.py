#!/usr/bin/env python3
from __future__ import annotations
import argparse, struct, zlib
from pathlib import Path

UF2_MAGIC0 = 0x0A324655
UF2_MAGIC1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
XIP_BASE = 0x10000000
ROM_SLOT_OFFSET = 4 * 1024 * 1024
ROM_SLOT_ADDR = XIP_BASE + ROM_SLOT_OFFSET
ROM_HEADER_SIZE = 256
ROM_SLOT_TOTAL = 4 * 1024 * 1024
ROM_MAX = ROM_SLOT_TOTAL - ROM_HEADER_SIZE
ROM_MAGIC = 0x52345843  # bytes C X 4 R
ROM_VERSION = 1


def read_uf2(path: Path):
    raw = path.read_bytes()
    if len(raw) % 512:
        raise SystemExit(f"UF2 invalido: tamanho {len(raw)} nao e multiplo de 512")
    blocks = []
    for i in range(0, len(raw), 512):
        b = bytearray(raw[i:i+512])
        m0,m1 = struct.unpack_from('<II', b, 0)
        mend, = struct.unpack_from('<I', b, 508)
        if (m0,m1,mend) != (UF2_MAGIC0,UF2_MAGIC1,UF2_MAGIC_END):
            raise SystemExit(f"UF2 invalido no bloco {i//512}")
        blocks.append(b)
    if not blocks:
        raise SystemExit("UF2 vazio")
    return blocks


def prepare_rom(path: Path) -> bytes:
    data = path.read_bytes()
    header = 0
    if len(data) % 0x8000 == 512:
        data = data[512:]
        header = 512
    if not data or len(data) > ROM_MAX:
        raise SystemExit(f"ROM com tamanho inesperado: {len(data)} bytes; maximo {ROM_MAX}")
    print(f"ROM: {path.name}")
    print(f"Copier header removido: {header} bytes")
    print(f"Tamanho: {len(data)} bytes")
    return data


def make_rom_blocks(rom: bytes, flags: int, family: int):
    crc = zlib.crc32(rom) & 0xffffffff
    hdr = struct.pack('<IIII', ROM_MAGIC, ROM_VERSION, len(rom), crc)
    slot = hdr + (b'\xff' * (ROM_HEADER_SIZE - len(hdr))) + rom
    if len(slot) % 256:
        slot += b'\xff' * (256 - (len(slot) % 256))
    out=[]
    for off in range(0, len(slot), 256):
        payload=slot[off:off+256]
        b=bytearray(512)
        struct.pack_into('<IIIIIIII', b, 0,
                         UF2_MAGIC0, UF2_MAGIC1, flags,
                         ROM_SLOT_ADDR + off, len(payload), 0, 0, family)
        b[32:32+len(payload)] = payload
        struct.pack_into('<I', b, 508, UF2_MAGIC_END)
        out.append(b)
    return out, crc


def main():
    ap=argparse.ArgumentParser(description='Injeta a ROM local do Mega Man X2/X3 em um UF2 generico CX4 v0.5.')
    ap.add_argument('base_uf2', type=Path)
    ap.add_argument('rom', type=Path)
    ap.add_argument('-o','--output', type=Path)
    ns=ap.parse_args()

    base=read_uf2(ns.base_uf2)
    rom=prepare_rom(ns.rom)

    # Refuse a base image that already occupies the reserved ROM slot.
    slot_lo=ROM_SLOT_ADDR
    slot_hi=ROM_SLOT_ADDR+ROM_SLOT_TOTAL
    for i,b in enumerate(base):
        target,payload=struct.unpack_from('<II', b, 12)
        if target < slot_hi and target + payload > slot_lo:
            raise SystemExit(f"O UF2 base invade o slot reservado de ROM no bloco {i}: 0x{target:08X}")

    flags, = struct.unpack_from('<I', base[0], 8)
    family, = struct.unpack_from('<I', base[0], 28)
    rom_blocks, crc = make_rom_blocks(rom, flags, family)
    all_blocks = base + rom_blocks
    total=len(all_blocks)
    for no,b in enumerate(all_blocks):
        struct.pack_into('<II', b, 20, no, total)

    out=ns.output or ns.base_uf2.with_name(ns.base_uf2.stem + '_COM_ROM.uf2')
    out.write_bytes(b''.join(all_blocks))
    print(f"CRC32: {crc:08X}")
    print(f"Slot flash: 0x{ROM_SLOT_OFFSET:06X} (XIP 0x{ROM_SLOT_ADDR:08X})")
    print(f"Blocos firmware: {len(base)}")
    print(f"Blocos ROM: {len(rom_blocks)}")
    print(f"Pronto: {out}")

if __name__ == '__main__':
    main()
