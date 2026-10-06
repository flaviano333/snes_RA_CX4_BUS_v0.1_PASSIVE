from pathlib import Path

ROM_SIZE = 0x8000  # 32 KiB LoROM, mapped at bank 00:$8000-$FFFF
BASE_ADDR = 0x8000

class A:
    def __init__(self):
        self.b=bytearray(); self.labels={}; self.fix=[]
    @property
    def pc(self): return BASE_ADDR+len(self.b)
    def emit(self,*xs): self.b.extend(x & 0xff for x in xs)
    def label(self,n): self.labels[n]=self.pc
    def bra(self,op,label):
        self.emit(op,0); self.fix.append((len(self.b)-1,label))
    def jmp(self,label):
        self.emit(0x4c,0,0); self.fix.append((len(self.b)-2,label,'abs'))
    def resolve(self):
        for item in self.fix:
            if len(item)==2:
                pos,label=item; target=self.labels[label]; here=BASE_ADDR+pos+1
                d=target-here
                if not -128<=d<=127: raise ValueError((label,d))
                self.b[pos]=d&0xff
            else:
                pos,label,_=item; target=self.labels[label]
                self.b[pos]=target&0xff; self.b[pos+1]=(target>>8)&0xff

a=A()
# Reset entry. Keep emulation mode; use 8-bit A/X and no interrupts.
a.emit(0x78)              # SEI
a.emit(0xD8)              # CLD
a.emit(0xA2,0xFF)         # LDX #$FF
a.emit(0x9A)              # TXS
a.emit(0xA9,0x00)         # LDA #0
a.emit(0x8D,0x00,0x42)    # STA $4200 (NMI/IRQ off)
a.emit(0x85,0x00,0x85,0x01,0x85,0x02,0x85,0x03)  # clear zp 00-03
# magic BUS6 at $7FF0-$7FF3
for addr,val in [(0x7ff0,ord('B')),(0x7ff1,ord('U')),(0x7ff2,ord('S')),(0x7ff3,ord('6'))]:
    a.emit(0xA9,val,0x8D,addr&0xff,addr>>8)

a.label('main')
a.emit(0xE6,0x02)         # INC sequence
a.emit(0xA5,0x02,0x8D,0xF4,0x7F)  # seq -> $7FF4
a.emit(0xA9,0x10,0x8D,0xF5,0x7F)  # phase WRITE

# Write 12 pages $6000-$6BFF. value = low ^ page ^ $5A.
for page in range(12):
    base=0x6000+page*0x100
    lab=f'w{page}'
    a.emit(0xA2,0x00)      # LDX #0
    a.label(lab)
    a.emit(0x8A)           # TXA
    a.emit(0x49,(page ^ 0x5A)&0xff) # EOR #imm
    a.emit(0x9D,base&0xff,base>>8)  # STA abs,X
    a.emit(0xE8)           # INX
    a.bra(0xD0,lab)        # BNE

# End write / begin register sequence
a.emit(0xA9,0x20,0x8D,0xF5,0x7F)
for addr,val in [(0x7f49,0x00),(0x7f4a,0x80),(0x7f4b,0x02),(0x7f4d,0x0e),(0x7f4e,0x00),(0x7f4f,0x5c)]:
    a.emit(0xA9,val,0x8D,addr&0xff,addr>>8)
a.emit(0xA9,0x30,0x8D,0xF5,0x7F)  # phase READ

# error count zp00/01 = 0
a.emit(0xA9,0x00,0x85,0x00,0x85,0x01)
for page in range(12):
    base=0x6000+page*0x100
    lab=f'r{page}'; ok=f'ok{page}'; noc=f'noc{page}'
    a.emit(0xA2,0x00)
    a.label(lab)
    a.emit(0xBD,base&0xff,base>>8)  # LDA abs,X
    a.emit(0x85,0x03)              # STA zp03
    a.emit(0x8A)                   # TXA
    a.emit(0x49,(page ^ 0x5A)&0xff)
    a.emit(0xC5,0x03)              # CMP zp03
    a.bra(0xF0,ok)                 # BEQ ok
    a.emit(0xE6,0x00)              # INC err lo
    a.bra(0xD0,noc)                # BNE nocarry
    a.emit(0xE6,0x01)              # INC err hi
    a.label(noc)
    a.label(ok)
    a.emit(0xE8)
    a.bra(0xD0,lab)

# report read errors and end pass
a.emit(0xA5,0x00,0x8D,0xF6,0x7F)
a.emit(0xA5,0x01,0x8D,0xF7,0x7F)
a.emit(0xA9,0x40,0x8D,0xF5,0x7F)
# short deterministic delay so terminal status can be sampled without insane pass rate
# Y isn't available in emulation? LDY/DEY fine, 8-bit. nested X/Y delay.
a.emit(0xA0,0x20)          # LDY #$20
a.label('delay_y')
a.emit(0xA2,0x00)          # LDX #0 => 256 loop due wrap
a.label('delay_x')
a.emit(0xE8)
a.bra(0xD0,'delay_x')
a.emit(0x88)               # DEY
a.bra(0xD0,'delay_y')
a.jmp('main')

a.resolve()
rom=bytearray([0xff])*ROM_SIZE
rom[:len(a.b)] = a.b
# LoROM header at $7FC0.
header=0x7fc0
title=b'CX4 BUS VALIDATOR V06 '
title=title[:21].ljust(21,b' ')
rom[header:header+21]=title
rom[header+0x15]=0x20  # LoROM slow
rom[header+0x16]=0x00  # ROM only
rom[header+0x17]=0x05  # 32 KiB
rom[header+0x18]=0x00  # SRAM none
rom[header+0x19]=0x01  # region USA
rom[header+0x1a]=0x00
rom[header+0x1b]=0x00
# vectors. Point all relevant vectors to reset entry to keep behavior deterministic.
for off in [0x7fe4,0x7fe6,0x7fe8,0x7fea,0x7fec,0x7fee,0x7ff4,0x7ff6,0x7ff8,0x7ffa,0x7ffc,0x7ffe]:
    rom[off]=BASE_ADDR&0xff; rom[off+1]=(BASE_ADDR>>8)&0xff
# checksum fields zero while calculating, then set complement/checksum.
rom[header+0x1c:header+0x20]=b'\x00\x00\x00\x00'
chk=sum(rom)&0xffff
comp=chk^0xffff
rom[header+0x1c]=comp&0xff; rom[header+0x1d]=comp>>8
rom[header+0x1e]=chk&0xff; rom[header+0x1f]=chk>>8
out=Path(__file__).with_name('CX4_BUS_TEST.sfc')
out.write_bytes(rom)
print(f'{out} {len(rom)} bytes code={len(a.b)} reset=${BASE_ADDR:04X} checksum={chk:04X}')
