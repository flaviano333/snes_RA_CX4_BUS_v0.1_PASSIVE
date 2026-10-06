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
ROM_MAGIC = 0x52345843
ROM_VERSION = 1


def read_uf2(path: Path):
    raw = path.read_bytes()
    if len(raw) % 512:
        raise SystemExit(f"UF2 invalido: tamanho {len(raw)} nao e multiplo de 512")
    blocks=[]
    for i in range(0,len(raw),512):
        b=bytearray(raw[i:i+512])
        m0,m1=struct.unpack_from('<II',b,0)
        mend,=struct.unpack_from('<I',b,508)
        if (m0,m1,mend)!=(UF2_MAGIC0,UF2_MAGIC1,UF2_MAGIC_END):
            raise SystemExit(f"UF2 invalido no bloco {i//512}")
        blocks.append(b)
    if not blocks: raise SystemExit("UF2 vazio")
    return blocks


def prepare_rom(path: Path)->bytes:
    data=path.read_bytes()
    header=0
    if len(data)%0x8000==512:
        data=data[512:]; header=512
    if not data or len(data)>ROM_MAX:
        raise SystemExit(f"ROM com tamanho inesperado: {len(data)} bytes; maximo {ROM_MAX}")
    print(f"ROM: {path.name}")
    print(f"Copier header removido: {header} bytes")
    print(f"Tamanho: {len(data)} bytes")
    return data


def make_slot(rom:bytes,flags:int,family:int):
    crc=zlib.crc32(rom)&0xffffffff
    hdr=struct.pack('<IIII',ROM_MAGIC,ROM_VERSION,len(rom),crc)
    slot=hdr+b'\xff'*(ROM_HEADER_SIZE-len(hdr))+rom
    if len(slot)%256: slot+=b'\xff'*(256-len(slot)%256)
    blocks=[]
    for off in range(0,len(slot),256):
        payload=slot[off:off+256]
        b=bytearray(512)
        struct.pack_into('<IIIIIIII',b,0,UF2_MAGIC0,UF2_MAGIC1,flags,
                         ROM_SLOT_ADDR+off,len(payload),0,0,family)
        b[32:32+len(payload)]=payload
        struct.pack_into('<I',b,508,UF2_MAGIC_END)
        blocks.append(b)
    total=len(blocks)
    for i,b in enumerate(blocks): struct.pack_into('<II',b,20,i,total)
    return blocks,crc


def main():
    ap=argparse.ArgumentParser(description='Cria um UF2 SOMENTE do slot persistente da ROM do jogo.')
    ap.add_argument('generic_uf2',type=Path,help='UF2 generico da firmware; usado apenas para copiar flags/family ID')
    ap.add_argument('rom',type=Path)
    ap.add_argument('-o','--output',type=Path,default=Path('MMX2_ROM_SLOT_UMA_VEZ.uf2'))
    ns=ap.parse_args()
    base=read_uf2(ns.generic_uf2)
    rom=prepare_rom(ns.rom)
    flags,=struct.unpack_from('<I',base[0],8)
    family,=struct.unpack_from('<I',base[0],28)
    blocks,crc=make_slot(rom,flags,family)
    ns.output.write_bytes(b''.join(blocks))
    print(f"CRC32: {crc:08X}")
    print(f"Slot persistente: flash +0x{ROM_SLOT_OFFSET:06X} (XIP 0x{ROM_SLOT_ADDR:08X})")
    print(f"Blocos ROM: {len(blocks)}")
    print(f"Pronto: {ns.output}")
    print("Grave este UF2 UMA VEZ. Builds genericas futuras nao tocam esse slot.")

if __name__=='__main__': main()
