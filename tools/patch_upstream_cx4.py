from pathlib import Path
import re, sys
p = Path(sys.argv[1])
s = p.read_text(encoding='utf-8')
s = s.replace('#include "saveload.h"\n', '')
# Bare-metal build: no host filesystem. Keep the bit-exact synthesizer and make
# cx4_load_firmware() simply synthesize the internal 1024x24 ROM.
pat = re.compile(r'static int cx4_verify_against_file\(.*?\n}\nint cx4_load_firmware\(.*?\n}\n', re.S)
rep = '''int cx4_load_firmware(Cx4 *c, const char *rom_path) {\n  (void)rom_path;\n  if (!c) return 0;\n  cx4_synthesize_data_rom(c);\n  return 1;\n}\n'''
s, n = pat.subn(rep, s, count=1)
if n != 1:
    raise SystemExit('nao consegui adaptar cx4_load_firmware no fonte upstream')
# Save-state hooks depend on the host emulator and are unused on RP2350.
s = re.sub(r'void cx4_saveload_clock\(.*?\n}\nvoid cx4_saveload\(.*?\n}\n', '', s, count=1, flags=re.S)
p.write_text(s, encoding='utf-8')
print('Core CX4 upstream adaptado para bare metal.')
