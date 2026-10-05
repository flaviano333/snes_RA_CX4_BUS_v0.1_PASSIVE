from pathlib import Path
import hashlib, sys

if len(sys.argv) != 3:
    raise SystemExit("uso: prepare_rom.py <Mega Man X2/X3 .sfc/.smc> <saida.bin>")
src, dst = map(Path, sys.argv[1:3])
data = src.read_bytes()
header = 0
if len(data) % 0x8000 == 512:
    data = data[512:]
    header = 512
if not data or len(data) > 4 * 1024 * 1024:
    raise SystemExit(f"ROM com tamanho inesperado: {len(data)} bytes")
dst.write_bytes(data)
print(f"ROM preparada: {src.name}")
print(f"Copier header removido: {header} bytes")
print(f"Tamanho embutido: {len(data)} bytes")
print(f"MD5: {hashlib.md5(data).hexdigest()}")
print(f"SHA256: {hashlib.sha256(data).hexdigest()}")
