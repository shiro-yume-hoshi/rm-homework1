"""PID live-tuning helper for STM32 targets.

The MCP server locks GDB's ``may-write-memory`` off, so a float gain cannot be
set with ``set var g_kp_pp = 420.0``. The only sanctioned write path is
``write_memory`` with a raw 32-bit word, which means the IEEE754 encoding has to
happen somewhere. This module is that somewhere, so no agent has to do it by
hand.

Addresses are resolved from the ELF at runtime (gdb first, a built-in ELF
symtab parser as fallback, an embedded table only as a last resort) so a
rebuild cannot silently invalidate them.

Subcommands:
    table                        list every tunable and observable
    resolve SYM                  address + type for SYM (members allowed)
    set SYM VALUE                validate + encode one parameter
    tune --pos Kp,Ki,Kd [--sp ...] [--pos-limits ...] [--sp-limits ...] [--ff N]
    kill                         emit the emergency stop write
    sample [--sec N]             emit a run_for_duration sampling call
    decode HEX                   decode a 32-bit word to float
    batch "g_kp_pp=420 g_ki_pp=.5"   (kept for compatibility)
    verify --axf ELF [--map M] [--no-fail]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import subprocess
import sys
from pathlib import Path

DEFAULT_ELF = r"D:\STM32project\Lesson4-PID\MDK-ARM\Lesson4_PID\Lesson4_PID.axf"
FLOAT = "float"

# name -> (role, validator key)
PARAMS: dict[str, tuple[str, str]] = {
    "g_kp_pp": ("position Kp", "gain"),
    "g_ki_pp": ("position Ki", "gain"),
    "g_kd_pp": ("position Kd", "gain"),
    "g_kp_sp": ("speed Kp", "gain"),
    "g_ki_sp": ("speed Ki", "gain"),
    "g_kd_sp": ("speed Kd", "gain"),
    "g_ff_scale": ("gravity feedforward scale", "ff_scale"),
    "g_ff_only": ("1 = feedforward only, gains forced to 0", "bool"),
    "g_imax_pp": ("position integrator limit", "nonneg"),
    "g_outmax_pp": ("position output limit (rpm)", "nonneg"),
    "g_dfilt_pp": ("position D filter", "dfilt"),
    "g_imax_sp": ("speed integrator limit", "nonneg"),
    "g_outmax_sp": ("speed output limit (control units)", "nonneg"),
    "g_dfilt_sp": ("speed D filter", "dfilt"),
}

OBSERVABLES: dict[str, tuple[str, str]] = {
    "stop_flag": ("uint8 safety kill, write 1 to stop", "uint8"),
    "g_tx_current": ("motor current A", FLOAT),
    "g_fdb_angle": ("feedback angle deg", FLOAT),
    "g_fdb_speed": ("feedback rpm", FLOAT),
    "g_tim6_cnt": ("TIM6 tick counter (loop is 1kHz)", "uint32"),
    "g_can_rx_cnt": ("CAN feedback count, >0 means motor is alive", "uint32"),
    "g_target_angle": ("target angle deg", FLOAT),
    "g_target_speed": ("target speed rpm", FLOAT),
    "g_intensity": ("final control output", FLOAT),
    "g_step_state": ("step high/low segment", "uint32"),
}

EMBEDDED: dict[str, tuple[int, str]] = {
    "stop_flag": (0x20000000, "uint8_t"),
    "g_tx_current": (0x20000004, "float"),
    "g_fdb_angle": (0x20000008, "float"),
    "g_fdb_speed": (0x2000000C, "float"),
    "g_tim6_cnt": (0x20000010, "uint32_t"),
    "g_target_angle": (0x20000018, "float"),
    "g_target_speed": (0x2000001C, "float"),
    "g_intensity": (0x20000020, "float"),
    "g_kp_pp": (0x20000024, "float"),
    "g_ki_pp": (0x20000028, "float"),
    "g_kd_pp": (0x2000002C, "float"),
    "g_kp_sp": (0x20000030, "float"),
    "g_ki_sp": (0x20000034, "float"),
    "g_kd_sp": (0x20000038, "float"),
    "g_ff_scale": (0x2000003C, "float"),
    "g_ff_only": (0x20000040, "float"),
    "g_imax_pp": (0x20000044, "float"),
    "g_outmax_pp": (0x20000048, "float"),
    "g_dfilt_pp": (0x2000004C, "float"),
    "g_imax_sp": (0x20000050, "float"),
    "g_outmax_sp": (0x20000054, "float"),
    "g_dfilt_sp": (0x20000058, "float"),
    "g_step_state": (0x2000005C, "uint32_t"),
    "g_can_rx_cnt": (0x20000114, "uint32_t"),
}

VALIDATORS = {
    "gain": lambda v: (math.isfinite(v) and 0.0 <= v <= 1e6,
                       "finite, 0..1e6 (a negative gain on an arm is a sign error)"),
    "ff_scale": lambda v: (math.isfinite(v) and 0.0 <= v <= 100.0, "finite, 0..100"),
    "bool": lambda v: (v in (0.0, 1.0), "exactly 0 or 1"),
    "dfilt": lambda v: (math.isfinite(v) and 0.0 <= v <= 1.0,
                        "finite, 0..1 (PID.cpp:37 is a first-order lowpass)"),
    "nonneg": lambda v: (math.isfinite(v) and 0.0 <= v <= 1e6, "finite, 0..1e6"),
    FLOAT: lambda v: (math.isfinite(v) and abs(v) <= 1e12, "finite"),
    "uint8": lambda v: (0 <= v <= 255, "integer 0..255"),
    "uint32": lambda v: (0 <= v <= 0xFFFFFFFF, "integer 0..2^32-1"),
}

INT_TYPES = {"uint8_t", "uint32_t", "int32_t", "int", "unsigned int", "?"}


class TuneError(Exception):
    pass


# --- encoding ---------------------------------------------------------------


def f32_bits(value: float) -> int:
    """IEEE754 single, little-endian, explicit -- never the host byte order."""
    return struct.unpack("<I", struct.pack("<f", value))[0]


def bits_to_f32(word: int) -> float:
    return struct.unpack("<f", struct.pack("<I", word & 0xFFFFFFFF))[0]


def hexword(value: float) -> str:
    return "0x%08X" % f32_bits(value)


# --- ELF symtab (pure python, no toolchain dependency) -----------------------


def read_symtab(elf: Path) -> dict[str, int]:
    data = elf.read_bytes()
    if data[:4] != b"\x7fELF" or data[4] != 1:
        raise ValueError("not a 32-bit little-endian ELF")
    shoff = struct.unpack_from("<I", data, 0x20)[0]
    shentsize, shnum = struct.unpack_from("<HH", data, 0x2E)
    if shoff == 0 or shentsize == 0:
        raise ValueError("no section headers (stripped?)")
    if shnum == 0:
        shnum = struct.unpack_from("<I", data, shoff + 0x14)[0]
    sections = []
    for i in range(shnum):
        base = shoff + i * shentsize
        name, stype, _flags, addr, offset, size, link, _info, _align, entsize = \
            struct.unpack_from("<IIIIIIIIII", data, base)
        sections.append(dict(name=name, type=stype, addr=addr, offset=offset,
                             size=size, link=link, entsize=entsize))
    out: dict[str, int] = {}
    for sec in sections:
        if sec["type"] != 2:  # SHT_SYMTAB
            continue
        strtab = sections[sec["link"]]
        strbase = strtab["offset"]
        count = sec["size"] // sec["entsize"]
        for i in range(count):
            off = sec["offset"] + i * sec["entsize"]
            st_name, st_value, st_size, st_info, _other, st_shndx = \
                struct.unpack_from("<IIIBBH", data, off)
            if st_shndx == 0 or st_value == 0 or st_size == 0:
                continue
            end = data.index(b"\x00", strbase + st_name)
            name = data[strbase + st_name:end].decode("ascii", "replace")
            if name:
                out[name] = st_value
    return out


# --- resolution -------------------------------------------------------------


class Resolver:
    def __init__(self, elf: str = DEFAULT_ELF):
        self.elf = Path(elf)
        self.symtab: dict[str, int] = {}
        self.symtab_error: str | None = None
        self.gdb_error: str | None = None
        self._gdb: dict[str, tuple[int, str]] = {}
        if self.elf.is_file():
            try:
                self.symtab = read_symtab(self.elf)
            except Exception as exc:
                self.symtab_error = str(exc)
        else:
            self.symtab_error = "ELF not found: %s" % self.elf

    def _gdb_batch(self, names: list[str]) -> None:
        pending = [n for n in names if n not in self.symtab and n not in EMBEDDED]
        if not pending:
            return
        gdb = _which("arm-none-eabi-gdb")
        if not gdb:
            self.gdb_error = "arm-none-eabi-gdb not on PATH"
            return
        args = [gdb, "-batch", "-nx", "--nx", str(self.elf)]
        for i, name in enumerate(pending):
            args += ["-ex", "echo \\n__P%03d__\\n" % i, "-ex", "p &%s" % name]
        try:
            proc = subprocess.run(args, capture_output=True, timeout=120)
        except Exception as exc:
            self.gdb_error = str(exc)
            return
        blob = (proc.stdout or b"").decode("latin-1", "replace")
        chunks = re.split(r"__P(\d{3})__", blob)
        for i in range(1, len(chunks) - 1, 2):
            try:
                idx = int(chunks[i])
            except ValueError:
                continue
            if not 0 <= idx < len(pending):
                continue
            m = re.search(r"\(\s*(.*?)\s*\*\s*\)\s*(0x[0-9a-fA-F]+)", chunks[i + 1])
            if m:
                self._gdb[pending[idx]] = (int(m.group(2), 16), _normalise_type(m.group(1)))

    def resolve(self, name: str) -> tuple[int, str, str]:
        """-> (address, c_type, source). Raises TuneError if unresolvable."""
        if name in self.symtab:
            return self.symtab[name], self._type_of(name), "elf-symtab"
        if name in EMBEDDED:
            return EMBEDDED[name][0], EMBEDDED[name][1], "embedded"
        if name in self._gdb:
            addr, ctype = self._gdb[name]
            return addr, ctype, "gdb"
        raise TuneError(
            "cannot resolve %r. Pass --axf pointing at a freshly built ELF, "
            "or check the symbol spelling. Members like motor.spid_.kp_ work "
            "too but need gdb." % name
        )

    def _type_of(self, name: str) -> str:
        _, ctype, _ = self._resolve_typed(name)
        return ctype

    def _resolve_typed(self, name: str) -> tuple[int, str, str]:
        if name in self._gdb:
            addr, ctype = self._gdb[name]
            return addr, ctype, "gdb"
        if name in self.symtab:
            if name in EMBEDDED:
                return self.symtab[name], EMBEDDED[name][1], "elf-symtab"
            return self.symtab[name], "?", "elf-symtab"
        if name in EMBEDDED:
            return EMBEDDED[name][0], EMBEDDED[name][1], "embedded"
        self._gdb_batch([name])
        if name in self._gdb:
            addr, ctype = self._gdb[name]
            return addr, ctype, "gdb"
        raise TuneError(
            "cannot resolve %r. Pass --axf pointing at a freshly built ELF, or "
            "check the spelling. Members such as motor.spid_.kp_ resolve via "
            "gdb, so they need arm-none-eabi-gdb on PATH." % name
        )

    def preload(self) -> None:
        self._gdb_batch(sorted(set(PARAMS) | set(OBSERVABLES)))


def _normalise_type(ctype: str) -> str:
    out = ctype.replace("volatile", "").strip()
    for alias, canon in (
        ("uint8_t", "uint8_t"), ("unsigned char", "uint8_t"),
        ("uint32_t", "uint32_t"), ("unsigned int", "uint32_t"),
        ("unsigned long", "uint32_t"), ("float", "float"),
        ("int", "int32_t"), ("long", "int32_t"),
    ):
        if out == alias:
            return canon
    return out


def _which(name: str) -> str | None:
    from shutil import which

    return which(name)


# --- validation -------------------------------------------------------------


def check(name: str, value: float, ctype: str) -> str:
    """Returns the encoding kind, or raises. Never guesses."""
    if name in OBSERVABLES:
        kind = OBSERVABLES[name][1]
        if kind == FLOAT:
            if ctype not in (FLOAT, "?"):
                raise TuneError(
                    "%s resolves to %s, not float" % (name, ctype))
        else:
            if ctype not in INT_TYPES:
                raise TuneError(
                    "%s resolves to %s, an integer type. Encoding it as float32 "
                    "would write 4 bytes and clobber its neighbours -- use "
                    "`pidmem.py kill` for stop_flag, read-only for the rest."
                    % (name, ctype))
            if float(value) != int(value):
                raise TuneError("%s is an integer field; %r is not" % (name, value))
            ok, why = VALIDATORS[kind](int(value))
            if not ok:
                raise TuneError("%s = %r rejected: must be %s" % (name, value, why))
            return "int"

    kind = PARAMS[name][1] if name in PARAMS else FLOAT
    if ctype not in (FLOAT, "?"):
        raise TuneError(
            "%s resolves to %s, not float. Refusing to encode a float32 bit "
            "pattern into a non-float field -- it would be interpreted as raw "
            "bits and produce a garbage value." % (name, ctype))
    ok, why = VALIDATORS[kind](value)
    if not ok:
        raise TuneError("%s = %r rejected: must be %s" % (name, value, why))
    return "float"


def _parse_triple(spec: str) -> tuple[float, float, float]:
    parts = [p for p in spec.split(",") if p != ""]
    if len(parts) != 3:
        raise TuneError("expected three comma-separated numbers, got %r" % spec)
    try:
        return tuple(float(p) for p in parts)  # type: ignore[return-value]
    except ValueError as exc:
        raise TuneError(str(exc)) from exc


def write_step(r: Resolver, name: str, value: float) -> dict:
    addr, ctype, source = r._resolve_typed(name)
    kind = check(name, value, ctype)
    if kind == "int":
        word = "0x%X" % int(value)
        shown = int(value)
    else:
        word = hexword(value)
        shown = value
    return {
        "tool": "write_memory",
        "args": {"address": "0x%08X" % addr, "value": word},
        "symbol": name,
        "encoded": kind,
        "float": value,
        "bits": word,
        "type": ctype,
        "role": PARAMS.get(name, OBSERVABLES.get(name, ("", "")))[0],
        "source": source,
        "_shown": shown,
    }


def emit(payload: dict, as_json: bool, trailer: str) -> None:
    printable = json.loads(json.dumps(payload))
    for s in printable.get("steps", []):
        shown = s.pop("_shown", None)
        if shown is not None and s.get("encoded") == "float":
            s["float"] = shown
        else:
            s.pop("_shown", None)
    print(json.dumps(printable, indent=2) if as_json else _render(payload))
    if trailer and not as_json:
        print()
        print(trailer)


def _render(payload: dict) -> str:
    lines = []
    for s in payload["steps"]:
        if "symbol" not in s:
            lines.append("%s(%s)" % (s.get("tool", "?"),
                                     json.dumps(s.get("args", {}), sort_keys=True)))
            continue
        label = repr(s["_shown"]) if "_shown" in s else ""
        lines.append("%-16s %s <- %-12s %-12s %s"
                     % (s["symbol"], s["args"]["address"], s["args"]["value"],
                        label, s.get("role", "")))
    return "\n".join(lines) if lines else "(empty payload)"


# --- commands ---------------------------------------------------------------


def cmd_table(args, r: Resolver) -> int:
    r.preload()
    rows = []
    drift = 0
    for name in sorted(set(PARAMS) | set(OBSERVABLES), key=lambda n: r._safe_addr(n)):
        addr, ctype, source = r._resolve_typed(name)
        embedded = EMBEDDED.get(name)
        status = ""
        if embedded and embedded[0] != addr:
            status = "  DRIFT embedded=0x%08X" % embedded[0]
            drift += 1
        rows.append({
            "symbol": name, "address": "0x%08X" % addr, "type": ctype,
            "source": source, "role": PARAMS.get(name, OBSERVABLES.get(name, ("", "")))[0],
            "writable": name in PARAMS, "status": status.strip(),
        })
    if args.json:
        print(json.dumps(rows, indent=2))
    else:
        for row in rows:
            print("%-16s %s  %-9s %-11s %-46s %s"
                  % (row["symbol"], row["address"], row["type"], row["source"],
                     row["role"], row["status"]))
        print()
        print("KILL:  write_memory(0x20000000, 1)   or   pidmem.py kill")
    if drift:
        print("WARNING: %d symbol(s) drifted from the embedded table. Run "
              "`pidmem.py verify --axf <elf>` -- or the Keil post-build will fail."
              % drift)
    return 0


def cmd_resolve(args, r: Resolver) -> int:
    addr, ctype, source = r._resolve_typed(args.symbol)
    out = {"symbol": args.symbol, "address": "0x%08X" % addr,
           "type": ctype, "source": source,
           "byte_order": "little",
           "role": PARAMS.get(args.symbol, OBSERVABLES.get(args.symbol, ("", "")))[0]}
    print(json.dumps(out, indent=2))
    return 0


def cmd_set(args, r: Resolver) -> int:
    step = write_step(r, args.symbol, float(args.value))
    emit({"steps": [step]}, args.json,
         "Then read_variable(%r) to confirm. A successful write is not proof of "
         "effect -- g_ff_only=1 zeroes the gains every cycle." % args.symbol)
    return 0


def cmd_tune(args, r: Resolver) -> int:
    steps: list[dict] = []
    if args.pos:
        kp, ki, kd = _parse_triple(args.pos)
        for n, v in zip(("g_kp_pp", "g_ki_pp", "g_kd_pp"), (kp, ki, kd)):
            steps.append(write_step(r, n, v))
    if args.sp:
        kp, ki, kd = _parse_triple(args.sp)
        for n, v in zip(("g_kp_sp", "g_ki_sp", "g_kd_sp"), (kp, ki, kd)):
            steps.append(write_step(r, n, v))
    if args.pos_limits:
        i, o, d = _parse_triple(args.pos_limits)
        for n, v in zip(("g_imax_pp", "g_outmax_pp", "g_dfilt_pp"), (i, o, d)):
            steps.append(write_step(r, n, v))
    if args.sp_limits:
        i, o, d = _parse_triple(args.sp_limits)
        for n, v in zip(("g_imax_sp", "g_outmax_sp", "g_dfilt_sp"), (i, o, d)):
            steps.append(write_step(r, n, v))
    if args.ff is not None:
        steps.append(write_step(r, "g_ff_scale", args.ff))
    if args.ff_only is not None:
        steps.append(write_step(r, "g_ff_only", float(args.ff_only)))
    if not steps:
        raise TuneError("nothing to do: pass --pos and/or --sp")
    steps.sort(key=lambda s: s["args"]["address"])
    emit({"steps": steps}, args.json,
         "Pass steps[] to the batch tool. NOTE these are separate 32-bit SWD\n"
         "writes, not one atomic transaction: the 1kHz ISR can observe a torn\n"
         "mix for a single cycle. Benign here, but not atomic.")
    return 0


def cmd_kill(args, r: Resolver) -> int:
    step = write_step(r, "stop_flag", 1.0)
    step["role"] = "emergency stop"
    emit({"steps": [step]}, args.json,
         "Emit this BEFORE investigating anything that moves the motor.")
    return 0


def cmd_sample(args, r: Resolver) -> int:
    exprs = args.expressions or [
        "g_tim6_cnt", "g_target_angle", "g_fdb_angle",
        "g_target_speed", "g_fdb_speed", "g_intensity", "g_can_rx_cnt",
    ]
    missing = [e for e in exprs if e not in PARAMS and e not in OBSERVABLES]
    if missing:
        raise TuneError("not symbols I know: %s" % ", ".join(missing))
    interval = args.interval_ms / 1000.0 if args.interval_ms else args.interval_sec
    call = {
        "tool": "run_for_duration",
        "args": {
            "duration_sec": args.sec,
            "resume_after": True,
            "sample": {"interval_sec": interval, "expressions": exprs},
        },
    }
    emit({"steps": [call]}, args.json,
         "sample polls while the target runs and does not disturb the 1kHz loop.\n"
         "resume_after=true leaves the arm running when the call returns.")
    return 0


def cmd_decode(args) -> int:
    print(bits_to_f32(int(args.word, 0)))


def cmd_batch(args, r: Resolver) -> int:
    steps = []
    for token in args.spec.split():
        if "=" not in token:
            raise TuneError("bad token %r, expected name=value" % token)
        name, raw = token.split("=", 1)
        steps.append(write_step(r, name, float(raw)))
    steps.sort(key=lambda s: s["args"]["address"])
    emit({"steps": steps}, args.json,
         "Prefer `tune` -- it validates each value before emitting anything.")
    return 0


def cmd_verify(args) -> int:
    """Resolve every symbol from the ELF, compare against the embedded table."""
    elf = Path(args.axf)
    if not elf.is_file():
        return _fail(args, "ELF not found: %s" % elf)

    print("verifying %s" % elf)
    r = Resolver(str(elf))
    if r.symtab_error and not r.symtab:
        return _fail(args, "could not read ELF symtab: %s" % r.symtab_error)
    print("  parsed %d symbols from the ELF symtab" % len(r.symtab))
    if args.map:
        _report_map(Path(args.map))

    drift = []
    unknown = []
    for name in sorted(set(PARAMS) | set(OBSERVABLES)):
        embedded = EMBEDDED.get(name)
        if embedded is None:
            unknown.append(name)
            continue
        if name not in r.symtab:
            drift.append((name, embedded[0], None, "not in ELF symtab"))
            continue
        actual = r.symtab[name]
        if actual != embedded[0]:
            drift.append((name, embedded[0], actual, "address"))
        elif elf.stat().st_size and args.check_type:
            pass

    for name, expected, actual, why in drift:
        got = "0x%08X" % actual if actual is not None else "MISSING"
        print("  DRIFT  %-16s expected 0x%08X  actual %-10s  (%s)"
              % (name, expected, got, why))
    for name in unknown:
        print("  NEW    %-16s in firmware, not in the embedded table" % name)

    if drift:
        print()
        print("%d symbol(s) drifted. Symptom if ignored: writes land on the wrong"
              % len(drift))
        print("address, the tool reports success, and the gain simply does not"
              " change.")
        print()
        print("Refresh the table with:")
        print("  pidmem.py table --axf %s" % elf)
        return _fail(args, "%d address drift(s)" % len(drift))

    print("  all %d symbols match the embedded table" % len(set(PARAMS) | set(OBSERVABLES)))
    if args.check_type:
        _check_types(elf)
    return 0


def _check_types(elf: Path) -> None:
    r = Resolver(str(elf))
    r.preload()
    bad = []
    for name in sorted(set(PARAMS) | set(OBSERVABLES)):
        try:
            _, ctype, _ = r._resolve_typed(name)
        except TuneError:
            continue
        want = EMBEDDED.get(name, (0, FLOAT))[1]
        if ctype not in ("?", "") and ctype != _normalise_type(want):
            bad.append((name, want, ctype))
    for name, want, got in bad:
        print("  TYPE   %-16s expected %-8s actual %s" % (name, want, got))
    if bad:
        print("  %d type mismatch(es) -- a float field was retyped, so "
              "write_memory would corrupt it." % len(bad))


def _report_map(mapfile: Path) -> None:
    if not mapfile.is_file():
        print("  (map not found at %s, skipping cross-check)" % mapfile)
        return
    try:
        text = mapfile.read_text(encoding="utf-8", errors="replace")
    except Exception as exc:
        print("  (map unreadable: %s)" % exc)
        return
    region = re.search(r"Execution Region RW_IRAM1.*?Size:\s*(0x[0-9a-fA-F]+)", text, re.S)
    print("  map RW_IRAM1 size %s" % (region.group(1) if region else "not found"))


def _fail(args, message: str) -> int:
    if getattr(args, "no_fail", False):
        print("WARNING: %s (--no-fail, build continues)" % message)
        return 0
    print("FAIL: %s" % message)
    return 2


def _safe_addr(self: Resolver, name: str) -> int:
    try:
        return self._resolve_typed(name)[0]
    except TuneError:
        return 0xFFFFFFFF


Resolver._safe_addr = _safe_addr


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--axf", default=DEFAULT_ELF, help="ELF/AXF to resolve symbols from")
    common.add_argument("--map", help="Keil .map, cross-checked when present")
    common.add_argument("--json", action="store_true", help="machine-readable output")
    ap.add_argument("--axf", default=DEFAULT_ELF, help=argparse.SUPPRESS)
    ap.add_argument("--map", help=argparse.SUPPRESS)
    ap.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("table", parents=[common])

    p = sub.add_parser("resolve", parents=[common]); p.add_argument("symbol")

    p = sub.add_parser("set", parents=[common])
    p.add_argument("symbol"); p.add_argument("value")

    p = sub.add_parser("tune", parents=[common])
    p.add_argument("--pos", help="position loop Kp,Ki,Kd")
    p.add_argument("--sp", help="speed loop Kp,Ki,Kd")
    p.add_argument("--pos-limits", help="position i_max,out_max,d_filter")
    p.add_argument("--sp-limits", help="speed i_max,out_max,d_filter")
    p.add_argument("--ff", type=float, help="gravity feedforward scale")
    p.add_argument("--ff-only", type=float, choices=[0.0, 1.0])

    sub.add_parser("kill", parents=[common])

    p = sub.add_parser("sample", parents=[common])
    p.add_argument("--sec", type=float, default=10.0)
    p.add_argument("--interval-sec", type=float, default=0.2)
    p.add_argument("--interval-ms", type=float)
    p.add_argument("--expressions", nargs="*")

    p = sub.add_parser("decode", parents=[common]); p.add_argument("word")

    p = sub.add_parser("batch", parents=[common]); p.add_argument("spec")

    p = sub.add_parser("verify", parents=[common])
    p.add_argument("--no-fail", action="store_true", help="warn but exit 0")
    p.add_argument("--check-type", action="store_true", help="also resolve DWARF types")

    args = ap.parse_args(argv)
    if args.cmd == "decode":
        return cmd_decode(args)
    if args.cmd == "verify":
        return cmd_verify(args)
    r = Resolver(args.axf)
    try:
        return {
            "table": cmd_table, "resolve": cmd_resolve, "set": cmd_set,
            "tune": cmd_tune, "kill": cmd_kill, "sample": cmd_sample,
            "batch": cmd_batch,
        }[args.cmd](args, r)
    except TuneError as exc:
        if args.json:
            print(json.dumps({"error": str(exc)}, indent=2))
        else:
            print("ERROR: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())