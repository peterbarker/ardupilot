#!/usr/bin/env python3

"""
Non-interactive crash dump analysis tool for ArduPilot.

Reads a crash_dump.bin file using GDB and CrashDebug, emitting a
human-readable report to stdout.

Usage:
    python3 crash_dump_report.py ELF_FILE CRASH_DUMP_FILE

Requires:
    - arm-none-eabi-gdb
    - CrashDebug binary (from modules/CrashDebug/bins/)

Copyright ArduPilot Project 2026
Released under GNU GPL version 3 or later
"""

import argparse
import os
import struct
import subprocess
import sys
import tempfile


# crash dump signature bytes
CRASHDUMP_SIG = b'cC'

# register names in dump order (version 3.0)
INT_REG_NAMES = [
    'R0', 'R1', 'R2', 'R3',
    'R4', 'R5', 'R6', 'R7',
    'R8', 'R9', 'R10', 'R11',
    'R12', 'SP', 'LR', 'PC',
    'xPSR', 'MSP', 'PSP', 'exceptionPSR',
]

FP_REG_NAMES = [f'S{i}' for i in range(32)] + ['FPSCR']

# Cortex-M fault status register addresses
FAULT_REGS = {
    'CFSR':   0xE000ED28,  # Configurable Fault Status Register
    'HFSR':   0xE000ED2C,  # HardFault Status Register
    'DFSR':   0xE000ED30,  # Debug Fault Status Register
    'MMFAR':  0xE000ED34,  # MemManage Fault Address Register
    'BFAR':   0xE000ED38,  # BusFault Address Register
    'AFSR':   0xE000ED3C,  # Auxiliary Fault Status Register
    'ICSR':   0xE000ED04,  # Interrupt Control and State Register
}

# CFSR bit definitions
CFSR_BITS = {
    # MemManage (bits 0-7)
    0:  'IACCVIOL  - Instruction access violation',
    1:  'DACCVIOL  - Data access violation',
    3:  'MUNSTKERR - MemManage fault on unstacking for return from exception',
    4:  'MSTKERR   - MemManage fault on stacking for exception entry',
    5:  'MLSPERR   - MemManage fault during FP lazy state preservation',
    7:  'MMARVALID - MMFAR holds a valid address',
    # BusFault (bits 8-15)
    8:  'IBUSERR   - Instruction bus error',
    9:  'PRECISERR - Precise data bus error',
    10: 'IMPRECISERR - Imprecise data bus error',
    11: 'UNSTKERR  - BusFault on unstacking for return from exception',
    12: 'STKERR    - BusFault on stacking for exception entry',
    13: 'LSPERR    - BusFault during FP lazy state preservation',
    15: 'BFARVALID - BFAR holds a valid address',
    # UsageFault (bits 16-31)
    16: 'UNDEFINSTR - Undefined instruction',
    17: 'INVSTATE  - Invalid state (e.g. executing ARM instruction in Thumb mode)',
    18: 'INVPC     - Invalid PC load',
    19: 'NOCP      - No coprocessor',
    20: 'STKOF     - Stack overflow (ARMv8-M)',
    24: 'UNALIGNED - Unaligned access',
    25: 'DIVBYZERO - Divide by zero',
}

HFSR_BITS = {
    1:  'VECTTBL - BusFault on vector table read',
    30: 'FORCED  - Forced HardFault (escalated from configurable fault)',
    31: 'DEBUGEVT - Debug event triggered HardFault',
}


def find_crashdebug():
    """Locate the CrashDebug binary."""
    script_dir = os.path.dirname(os.path.realpath(__file__))
    base = os.path.join(script_dir, '..', '..', 'modules', 'CrashDebug', 'bins')

    if sys.platform.startswith('linux'):
        path = os.path.join(base, 'lin64', 'CrashDebug')
    elif sys.platform == 'darwin':
        path = os.path.join(base, 'osx64', 'CrashDebug')
    elif sys.platform == 'win32':
        path = os.path.join(base, 'win32', 'CrashDebug.exe')
    else:
        return None

    path = os.path.normpath(path)
    if os.path.isfile(path):
        return path
    return None


def validate_crashdump(path):
    """Read and validate the crash dump header, return parsed info."""
    with open(path, 'rb') as f:
        data = f.read()

    if len(data) < 4:
        print(f"Error: crash dump too small ({len(data)} bytes)", file=sys.stderr)
        sys.exit(1)

    if data[0:2] != CRASHDUMP_SIG:
        print(f"Error: invalid crash dump signature (got 0x{data[0]:02x} 0x{data[1]:02x}, "
              f"expected 0x63 0x43)", file=sys.stderr)
        sys.exit(1)

    version_major = data[2]
    version_minor = data[3]

    if len(data) < 8:
        print("Error: crash dump truncated", file=sys.stderr)
        sys.exit(1)

    flags = struct.unpack_from('<I', data, 4)[0]
    has_fpu = bool(flags & 1)

    # parse integer registers
    int_reg_count = 20 if version_major >= 3 else 18
    reg_offset = 8
    needed = reg_offset + int_reg_count * 4
    if len(data) < needed:
        print(f"Error: crash dump truncated (need {needed} bytes for registers, "
              f"have {len(data)})", file=sys.stderr)
        sys.exit(1)

    int_regs = {}
    for i in range(int_reg_count):
        val = struct.unpack_from('<I', data, reg_offset + i * 4)[0]
        if i < len(INT_REG_NAMES):
            int_regs[INT_REG_NAMES[i]] = val

    offset = reg_offset + int_reg_count * 4

    # parse FP registers if present
    fp_regs = {}
    if has_fpu:
        fp_needed = offset + 33 * 4
        if len(data) >= fp_needed:
            for i in range(33):
                val = struct.unpack_from('<I', data, offset + i * 4)[0]
                fp_regs[FP_REG_NAMES[i]] = val
            offset += 33 * 4

    # parse memory regions
    regions = []
    while offset + 8 <= len(data):
        start_addr = struct.unpack_from('<I', data, offset)[0]
        end_addr = struct.unpack_from('<I', data, offset + 4)[0]
        if start_addr == 0xFFFFFFFF:
            break
        if start_addr == 0xACCE55ED:
            regions.append(('STACK_OVERFLOW_SENTINEL', 0, 0))
            break
        size = end_addr - start_addr
        offset += 8
        if offset + size > len(data):
            regions.append((start_addr, end_addr, min(size, len(data) - offset)))
            break
        regions.append((start_addr, end_addr, size))
        offset += size

    return {
        'version': (version_major, version_minor),
        'flags': flags,
        'has_fpu': has_fpu,
        'int_regs': int_regs,
        'fp_regs': fp_regs,
        'regions': regions,
        'size': len(data),
    }


def decode_fault_type(xpsr):
    """Decode the exception number from xPSR."""
    exception_num = xpsr & 0x1FF
    fault_names = {
        0: 'Thread mode (no exception)',
        2: 'NMI',
        3: 'HardFault',
        4: 'MemManage',
        5: 'BusFault',
        6: 'UsageFault',
        11: 'SVCall',
        12: 'Debug Monitor',
        14: 'PendSV',
        15: 'SysTick',
    }
    if exception_num in fault_names:
        return fault_names[exception_num]
    if exception_num >= 16:
        return f'IRQ{exception_num - 16} (External Interrupt)'
    return f'Reserved (exception #{exception_num})'


def build_gdb_script(int_regs):
    """Build the GDB commands for extracting crash information."""
    commands = []
    commands.append('set pagination off')
    commands.append('set print pretty on')
    commands.append('set print array on')
    commands.append('set print demangle on')
    commands.append('set width 0')

    # register dump
    commands.append('echo ===GDB_REGISTERS_START===\\n')
    commands.append('info registers')
    commands.append('echo ===GDB_REGISTERS_END===\\n')

    # resolve address-bearing registers to symbols
    # use "info symbol" for symbol+offset and "info line" for source location
    commands.append('echo ===GDB_SYMBOLS_START===\\n')
    for reg_name in ['PC', 'LR', 'R0', 'R1', 'R2', 'R3', 'R4', 'R5',
                     'R6', 'R7', 'R8', 'R9', 'R10', 'R11', 'R12']:
        addr = int_regs.get(reg_name)
        if addr is None:
            continue
        commands.append(f'echo {reg_name}_SYM=\\n')
        commands.append(f'info symbol {addr:#010x}')
        if reg_name in ('PC', 'LR'):
            commands.append(f'echo {reg_name}_LINE=\\n')
            commands.append(f'info line *{addr:#010x}')
    commands.append('echo ===GDB_SYMBOLS_END===\\n')

    # backtrace
    commands.append('echo ===GDB_BACKTRACE_START===\\n')
    commands.append('bt full')
    commands.append('echo ===GDB_BACKTRACE_END===\\n')

    # thread info
    commands.append('echo ===GDB_THREADS_START===\\n')
    commands.append('info threads')
    commands.append('echo ===GDB_THREADS_END===\\n')

    # try to read fault status registers - these are memory-mapped peripherals
    # and may or may not be available in the crash dump
    commands.append('echo ===GDB_FAULT_REGS_START===\\n')
    for name, addr in FAULT_REGS.items():
        commands.append(f'echo {name}=\\n')
        commands.append(f'x/1xw {addr:#010x}')
    commands.append('echo ===GDB_FAULT_REGS_END===\\n')

    # stack dump around SP
    commands.append('echo ===GDB_STACK_START===\\n')
    commands.append('x/32xw $sp')
    commands.append('echo ===GDB_STACK_END===\\n')

    commands.append('quit')
    return '\n'.join(commands)


def run_gdb(elf_file, dump_file, crashdebug_exe, int_regs):
    """Run GDB in batch mode and capture output."""
    script = build_gdb_script(int_regs)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.gdb', delete=False) as f:
        f.write(script)
        script_path = f.name

    try:
        target_cmd = (f'target remote | {crashdebug_exe} '
                      f'--elf {elf_file} --dump {dump_file}')
        cmd = [
            'arm-none-eabi-gdb',
            '-nx',
            '--batch',
            '--quiet',
            elf_file,
            '-ex', 'set target-charset ASCII',
            '-ex', target_cmd,
            '-x', script_path,
        ]

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=30,
        )
        return result.stdout, result.stderr, result.returncode
    except FileNotFoundError:
        print("Error: arm-none-eabi-gdb not found. Is the ARM toolchain installed?",
              file=sys.stderr)
        sys.exit(1)
    except subprocess.TimeoutExpired:
        print("Error: GDB timed out after 30 seconds", file=sys.stderr)
        sys.exit(1)
    finally:
        os.unlink(script_path)


def extract_section(output, start_marker, end_marker):
    """Extract text between markers from GDB output."""
    start = output.find(start_marker)
    end = output.find(end_marker)
    if start == -1 or end == -1:
        return None
    return output[start + len(start_marker):end].strip()


def parse_gdb_fault_regs(section):
    """Parse fault register values from GDB output."""
    if section is None:
        return {}
    regs = {}
    current_name = None
    for line in section.splitlines():
        line = line.strip()
        if line.endswith('='):
            current_name = line[:-1]
        elif current_name and ':' in line:
            # line like "0xe000ed28:   0x00000000"
            parts = line.split()
            if len(parts) >= 2:
                try:
                    regs[current_name] = int(parts[1], 16)
                except ValueError:
                    pass
            current_name = None
    return regs


def parse_gdb_symbols(section):
    """Parse symbol resolution results from GDB output.

    Returns a dict mapping register names to dicts with keys:
        'symbol': e.g. "AP_InertialSensor::update() + 124"
        'line':   e.g. "Line 456 of \"AP_InertialSensor.cpp\" ..."  (PC/LR only)
    """
    if section is None:
        return {}
    results = {}
    current_key = None
    current_field = None  # 'symbol' or 'line'
    for line in section.splitlines():
        line = line.strip()
        if line.endswith('_SYM='):
            reg_name = line[:-5]
            current_key = reg_name
            current_field = 'symbol'
            results.setdefault(reg_name, {})
        elif line.endswith('_LINE='):
            reg_name = line[:-6]
            current_key = reg_name
            current_field = 'line'
            results.setdefault(reg_name, {})
        elif current_key and current_field:
            # GDB outputs "No symbol matches ..." for unknown addresses
            if 'No symbol' in line or 'No line' in line:
                current_field = None
                continue
            results[current_key][current_field] = line
            current_field = None
    return results


def format_reg_with_symbol(name, value, symbols):
    """Format a register value with its symbolic name if available."""
    sym_info = symbols.get(name, {})
    sym = sym_info.get('symbol', '')
    line = sym_info.get('line', '')

    result = f'{name:>3}: {value:#010x}'
    if sym:
        # "info symbol" output looks like:
        #   "func_name + offset in section .text of /path/to/elf"
        # trim the " in section .text of /path" part for readability
        display = sym
        in_section = display.find(' in section ')
        if in_section != -1:
            display = display[:in_section]
        result += f'  <{display}>'
    if line:
        # "info line" output looks like:
        #   "Line 123 of \"file.cpp\" starts at address 0x... and ends at 0x..."
        # extract just the "Line N of "file.cpp"" part
        ends_at = line.find(' starts at address')
        if ends_at != -1:
            display = line[:ends_at]
        else:
            display = line
        # clean up escaped quotes from GDB
        display = display.replace('\\"', '"')
        result += f'\n{"":>18}{display}'
    return result


def decode_cfsr(val):
    """Decode CFSR register bits."""
    lines = []
    for bit, desc in sorted(CFSR_BITS.items()):
        if val & (1 << bit):
            lines.append(f'    bit {bit:2d}: {desc}')
    return lines


def decode_hfsr(val):
    """Decode HFSR register bits."""
    lines = []
    for bit, desc in sorted(HFSR_BITS.items()):
        if val & (1 << bit):
            lines.append(f'    bit {bit:2d}: {desc}')
    return lines


def print_report(dump_info, gdb_stdout, gdb_stderr):
    """Print the formatted crash dump report."""
    print('=' * 72)
    print('ArduPilot Crash Dump Report')
    print('=' * 72)

    # dump file info
    ver = dump_info['version']
    print(f'\nDump format version: {ver[0]}.{ver[1]}')
    print(f'Dump size: {dump_info["size"]} bytes')
    print(f'FPU state saved: {"yes" if dump_info["has_fpu"] else "no"}')

    # fault type from xPSR
    int_regs = dump_info['int_regs']
    if 'exceptionPSR' in int_regs:
        fault_type = decode_fault_type(int_regs['exceptionPSR'])
        exc_num = int_regs['exceptionPSR'] & 0x1FF
        print(f'Exception type: {fault_type} (exception #{exc_num})')
    elif 'xPSR' in int_regs:
        fault_type = decode_fault_type(int_regs['xPSR'])
        exc_num = int_regs['xPSR'] & 0x1FF
        print(f'Exception type: {fault_type} (exception #{exc_num})')

    # check for stack overflow sentinel
    for region in dump_info['regions']:
        if region[0] == 'STACK_OVERFLOW_SENTINEL':
            print('\n*** WARNING: CrashCatcher stack overflow detected! ***')
            print('*** The crash dump itself may be corrupt. ***')

    # resolve register addresses to symbols
    sym_section = extract_section(
        gdb_stdout, '===GDB_SYMBOLS_START===', '===GDB_SYMBOLS_END===')
    symbols = parse_gdb_symbols(sym_section)

    # registers from dump with symbolic names
    print(f'\n{"Registers":=^72}')
    print(f'  {format_reg_with_symbol("PC", int_regs.get("PC", 0), symbols)}')
    print(f'  {format_reg_with_symbol("LR", int_regs.get("LR", 0), symbols)}')
    print(f'  SP:  {int_regs.get("SP", 0):#010x}    '
          f'MSP: {int_regs.get("MSP", 0):#010x}    '
          f'PSP: {int_regs.get("PSP", 0):#010x}')
    # print R0-R12: those with symbols get their own line,
    # the rest are grouped compactly
    printed = set()
    for i in range(13):
        name = f'R{i}'
        if symbols.get(name, {}).get('symbol'):
            print(f'  {format_reg_with_symbol(name, int_regs.get(name, 0), symbols)}')
            printed.add(i)
    # print remaining in groups of 4
    non_sym = [i for i in range(13) if i not in printed]
    for start in range(0, len(non_sym), 4):
        group = non_sym[start:start + 4]
        parts = [f'R{i:>2}: {int_regs.get(f"R{i}", 0):#010x}' for i in group]
        print(f'  {"    ".join(parts)}')
    print(f'  xPSR: {int_regs.get("xPSR", 0):#010x}    '
          f'exceptionPSR: {int_regs.get("exceptionPSR", 0):#010x}')

    # fault status registers (from GDB/memory)
    fault_regs_section = extract_section(
        gdb_stdout, '===GDB_FAULT_REGS_START===', '===GDB_FAULT_REGS_END===')
    fault_regs = parse_gdb_fault_regs(fault_regs_section)

    if fault_regs:
        print(f'\n{"Fault Status Registers":=^72}')
        for name in ['CFSR', 'HFSR', 'DFSR', 'MMFAR', 'BFAR', 'AFSR', 'ICSR']:
            if name in fault_regs:
                print(f'  {name:6s}: {fault_regs[name]:#010x}')
                if name == 'CFSR' and fault_regs[name]:
                    for line in decode_cfsr(fault_regs[name]):
                        print(line)
                elif name == 'HFSR' and fault_regs[name]:
                    for line in decode_hfsr(fault_regs[name]):
                        print(line)
    else:
        print(f'\n{"Fault Status Registers":=^72}')
        print('  (not available in crash dump - SCB registers were not captured)')

    # backtrace from GDB
    bt_section = extract_section(
        gdb_stdout, '===GDB_BACKTRACE_START===', '===GDB_BACKTRACE_END===')
    print(f'\n{"Backtrace":=^72}')
    if bt_section:
        print(bt_section)
    else:
        print('  (GDB backtrace not available)')

    # threads from GDB
    threads_section = extract_section(
        gdb_stdout, '===GDB_THREADS_START===', '===GDB_THREADS_END===')
    if threads_section:
        print(f'\n{"Threads":=^72}')
        print(threads_section)

    # GDB register view (may contain symbolic names)
    regs_section = extract_section(
        gdb_stdout, '===GDB_REGISTERS_START===', '===GDB_REGISTERS_END===')
    if regs_section:
        print(f'\n{"Registers (GDB view)":=^72}')
        print(regs_section)

    # stack dump
    stack_section = extract_section(
        gdb_stdout, '===GDB_STACK_START===', '===GDB_STACK_END===')
    if stack_section:
        print(f'\n{"Stack Dump (32 words from SP)":=^72}')
        print(stack_section)

    # memory regions summary
    print(f'\n{"Memory Regions in Dump":=^72}')
    total_mem = 0
    for region in dump_info['regions']:
        if region[0] == 'STACK_OVERFLOW_SENTINEL':
            print('  [STACK OVERFLOW SENTINEL]')
            continue
        start, end, size = region
        total_mem += size
        print(f'  {start:#010x} - {end:#010x}  ({size:>7d} bytes)')
    print(f'  Total memory captured: {total_mem} bytes')

    # FP registers if present
    if dump_info['fp_regs']:
        print(f'\n{"Floating Point Registers":=^72}')
        fp = dump_info['fp_regs']
        # detect ChibiOS stack fill pattern (lazy FPU stacking)
        fill_pattern = 0x55555555
        s_regs = [fp.get(f'S{i}', 0) for i in range(32)]
        fill_count = sum(1 for v in s_regs if v == fill_pattern)
        if fill_count > 24:
            print('  FP registers contain ChibiOS stack fill pattern (0x55555555)')
            print('  This indicates lazy FPU context save - the faulting code was')
            print('  not using floating point, so the FP register slots on the')
            print('  stack were never written by hardware.')
            non_fill = {f'S{i}': v for i, v in enumerate(s_regs) if v != fill_pattern}
            if non_fill:
                print('  Non-fill registers:')
                for name, raw in non_fill.items():
                    fval = struct.unpack('<f', struct.pack('<I', raw))[0]
                    print(f'    {name}: {raw:#010x} ({fval:.6g})')
        else:
            for i in range(0, 32, 4):
                parts = []
                for j in range(4):
                    name = f'S{i+j}'
                    if name in fp:
                        raw = fp[name]
                        fval = struct.unpack('<f', struct.pack('<I', raw))[0]
                        parts.append(f'{name:>3}: {raw:#010x} ({fval:>12.6g})')
                print(f'  {"  ".join(parts)}')
        if 'FPSCR' in fp:
            print(f'  FPSCR: {fp["FPSCR"]:#010x}')

    # any GDB errors worth noting
    if gdb_stderr:
        # filter out common noise
        errors = []
        for line in gdb_stderr.splitlines():
            line = line.strip()
            if not line:
                continue
            # skip common harmless warnings
            if 'warning: No executable has been specified' in line:
                continue
            if 'Try "help target"' in line:
                continue
            errors.append(line)
        if errors:
            print(f'\n{"GDB Warnings/Errors":=^72}')
            for line in errors:
                print(f'  {line}')

    print(f'\n{"=" * 72}')
    print('Note: crash dumps contain only a subset of RAM at the time of')
    print('the fault. Some memory reads and backtraces may be incomplete.')
    print(f'{"=" * 72}')


def main():
    parser = argparse.ArgumentParser(
        description='Generate a non-interactive crash dump report for ArduPilot',
    )
    parser.add_argument('elf_file', help='ELF file matching the firmware that crashed')
    parser.add_argument('dump_file', help='crash_dump.bin file')
    parser.add_argument('--gdb', default='arm-none-eabi-gdb',
                        help='path to arm-none-eabi-gdb (default: from PATH)')
    parser.add_argument('--crashdebug', default=None,
                        help='path to CrashDebug binary (default: auto-detect)')

    args = parser.parse_args()

    if not os.path.isfile(args.elf_file):
        print(f"Error: ELF file not found: {args.elf_file}", file=sys.stderr)
        sys.exit(1)

    if not os.path.isfile(args.dump_file):
        print(f"Error: crash dump file not found: {args.dump_file}", file=sys.stderr)
        sys.exit(1)

    crashdebug = args.crashdebug or find_crashdebug()
    if crashdebug is None or not os.path.isfile(crashdebug):
        print("Error: CrashDebug binary not found. Ensure modules/CrashDebug "
              "is checked out.", file=sys.stderr)
        sys.exit(1)

    # validate and parse the dump file directly
    dump_info = validate_crashdump(args.dump_file)

    # run GDB for symbolic information
    gdb_stdout, gdb_stderr, gdb_rc = run_gdb(
        args.elf_file, args.dump_file, crashdebug, dump_info['int_regs'])

    print_report(dump_info, gdb_stdout, gdb_stderr)


if __name__ == '__main__':
    main()
