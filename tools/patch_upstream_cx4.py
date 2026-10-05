from pathlib import Path
import re
import sys

if len(sys.argv) != 2:
    raise SystemExit("uso: patch_upstream_cx4.py <third_party/cx4.c>")

p = Path(sys.argv[1])
s = p.read_text(encoding="utf-8")

# The upstream desktop core includes the emulator's save-state header. The RP2350
# build does not use save states, so remove only that dependency.
s = re.sub(r'^\s*#include\s+["<]saveload\.h[">]\s*\r?\n', '', s, flags=re.M)


def _find_matching_brace(src: str, open_pos: int) -> int:
    """Return the index of the closing brace matching src[open_pos].

    This tiny C-aware scanner skips strings, character literals and comments,
    so braces inside comments/format strings don't confuse the patcher.
    """
    if open_pos < 0 or src[open_pos] != '{':
        raise ValueError("open_pos does not point at '{'")
    depth = 0
    i = open_pos
    n = len(src)
    state = "code"
    while i < n:
        c = src[i]
        nxt = src[i + 1] if i + 1 < n else ''
        if state == "code":
            if c == '/' and nxt == '/':
                state = "line_comment"; i += 2; continue
            if c == '/' and nxt == '*':
                state = "block_comment"; i += 2; continue
            if c == '"':
                state = "string"; i += 1; continue
            if c == "'":
                state = "char"; i += 1; continue
            if c == '{':
                depth += 1
            elif c == '}':
                depth -= 1
                if depth == 0:
                    return i
        elif state == "line_comment":
            if c == '\n': state = "code"
        elif state == "block_comment":
            if c == '*' and nxt == '/':
                state = "code"; i += 2; continue
        elif state in ("string", "char"):
            quote = '"' if state == "string" else "'"
            if c == '\\':
                i += 2; continue
            if c == quote:
                state = "code"
        i += 1
    raise ValueError("unbalanced braces")


def _function_span(src: str, name: str):
    # Match the function definition by name, independent of return type, spacing,
    # comments or signature formatting. A declaration ending in ';' is skipped.
    pat = re.compile(r'\b' + re.escape(name) + r'\s*\(')
    for m in pat.finditer(src):
        par = 1
        i = m.end()
        state = "code"
        while i < len(src) and par:
            c = src[i]
            nxt = src[i + 1] if i + 1 < len(src) else ''
            if state == "code":
                if c == '/' and nxt == '/': state = "line_comment"; i += 2; continue
                if c == '/' and nxt == '*': state = "block_comment"; i += 2; continue
                if c == '"': state = "string"; i += 1; continue
                if c == "'": state = "char"; i += 1; continue
                if c == '(': par += 1
                elif c == ')': par -= 1
            elif state == "line_comment":
                if c == '\n': state = "code"
            elif state == "block_comment":
                if c == '*' and nxt == '/': state = "code"; i += 2; continue
            elif state in ("string", "char"):
                quote = '"' if state == "string" else "'"
                if c == '\\': i += 2; continue
                if c == quote: state = "code"
            i += 1
        if par:
            continue
        j = i
        while j < len(src) and src[j].isspace(): j += 1
        if j >= len(src) or src[j] != '{':
            continue  # prototype/call, not a definition
        close = _find_matching_brace(src, j)
        # Include the whole declaration line. This preserves preceding comments.
        start = src.rfind('\n', 0, m.start()) + 1
        end = close + 1
        if end < len(src) and src[end] == '\r': end += 1
        if end < len(src) and src[end] == '\n': end += 1
        return start, end
    return None


def replace_function(src: str, name: str, replacement: str, required=True):
    span = _function_span(src, name)
    if span is None:
        if required:
            raise SystemExit(f"nao encontrei a definicao de {name} no fonte upstream")
        return src, False
    a, b = span
    return src[:a] + replacement.rstrip() + "\n" + src[b:], True


def remove_function(src: str, name: str, required=False):
    return replace_function(src, name, "", required=required)

# On bare metal the 1024x24 HG51B data ROM is synthesized directly. There is
# no filesystem/environment from which to cross-check an optional cx4.rom.
load_fw = r'''int cx4_load_firmware(Cx4 *c, const char *rom_path) {
  (void)rom_path;
  if (!c) return 0;
  cx4_synthesize_data_rom(c);
  return 1;
}'''

s, _ = replace_function(s, "cx4_load_firmware", load_fw, required=True)
# Remove the now-unused file based verifier, if this upstream revision has it.
s, _ = remove_function(s, "cx4_verify_against_file", required=False)
# Save-state callbacks belong to the desktop host and are unused by this firmware.
s, _ = remove_function(s, "cx4_saveload_clock", required=False)
s, _ = remove_function(s, "cx4_saveload", required=False)

# Sanity checks against the API this project actually depends on. If upstream
# makes an incompatible API change, fail here with a useful message instead of
# producing mysterious compiler errors later.
required_symbols = [
    "cx4_create", "cx4_reset", "cx4_sync", "cx4_read", "cx4_write", "cx4_ram_ptr",
    "cx4_synthesize_data_rom", "cx4_firmware_loaded",
    "cx4_instructions_executed", "cx4_rdrom_hits", "cx4_run_ring_count",
    "cx4_locked",
]
missing = [name for name in required_symbols if not re.search(r'\b' + re.escape(name) + r'\s*\(', s)]
if missing:
    raise SystemExit("API CX4 upstream incompativel; faltando: " + ", ".join(missing))

p.write_text(s, encoding="utf-8", newline="\n")
print("Core CX4 upstream adaptado para bare metal (patch estrutural v0.3.4).")
