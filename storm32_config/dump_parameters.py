#!/usr/bin/env python3
"""
SToRM32 Parameter Dump Utility

Reads all parameters from gimbal and displays them with descriptions.
Can output in various formats: text, markdown, CSV, JSON.
"""

import sys
import json
import csv
import logging
from pathlib import Path
from typing import Dict, Any, Optional

sys.path.insert(0, str(Path(__file__).parent))

from core.serial_protocol import SerialProtocol  # noqa: E402
from core.parameters import ParameterManager  # noqa: E402
from models.parameter_definitions import PARAMETERS  # noqa: E402

logging.basicConfig(
    level=logging.WARNING,
    format='%(levelname)s: %(message)s'
)


def format_parameter_description(param_def: Dict[str, Any]) -> str:
    """
    Create a human-readable description from parameter definition.

    Args:
        param_def: Parameter definition dictionary

    Returns:
        Formatted description string
    """
    parts = []

    # Type
    param_type = param_def.get('type', 'UNKNOWN')
    parts.append(f"Type: {param_type}")

    # Range for numeric types
    if param_type in ['UINT', 'INT']:
        min_val = param_def.get('min')
        max_val = param_def.get('max')
        if min_val is not None and max_val is not None:
            # Apply decimal position if present
            ppos = param_def.get('ppos', 0)
            if ppos > 0:
                divisor = 10 ** ppos
                min_val = min_val / divisor
                max_val = max_val / divisor
            parts.append(f"Range: {min_val}-{max_val}")

    # Choices for LIST type
    elif param_type == 'LIST':
        choices = param_def.get('choices', [])
        if choices:
            if len(choices) <= 5:
                parts.append(f"Choices: {', '.join(choices)}")
            else:
                parts.append(f"Choices: {len(choices)} options ({choices[0]}, {choices[1]}, ...)")

    # Default value
    default = param_def.get('default')
    if default is not None:
        ppos = param_def.get('ppos', 0)
        if ppos > 0:
            default = default / (10 ** ppos)

        # For LIST type, show choice name
        if param_type == 'LIST':
            choices = param_def.get('choices', [])
            if 0 <= default < len(choices):
                parts.append(f"Default: {choices[int(default)]}")
            else:
                parts.append(f"Default: {default}")
        else:
            parts.append(f"Default: {default}")

    # Unit
    unit = param_def.get('unit', '')
    if unit:
        parts.append(f"Unit: {unit}")

    # Page/location
    page = param_def.get('page', '')
    if page:
        parts.append(f"Page: {page}")

    # Address
    adr = param_def.get('adr')
    if adr is not None:
        parts.append(f"Address: {adr}")

    return ' | '.join(parts)


def dump_parameters_text(manager: ParameterManager, output_file: Optional[Path] = None):
    """Dump parameters in human-readable text format."""

    lines = []
    lines.append("=" * 80)
    lines.append("SToRM32 PARAMETER DUMP")
    lines.append("=" * 80)
    lines.append("")

    # Group by page
    pages = {}
    for param_def in PARAMETERS:
        page = param_def.get('page', 'other')
        if page not in pages:
            pages[page] = []
        pages[page].append(param_def)

    # Sort pages
    sorted_pages = sorted(pages.items())

    for page_name, params in sorted_pages:
        lines.append(f"\n{'─' * 80}")
        lines.append(f"PAGE: {page_name.upper()}")
        lines.append(f"{'─' * 80}\n")

        for param_def in params:
            name = param_def.get('name', 'UNKNOWN')
            adr = param_def.get('adr')

            lines.append(f"Parameter: {name}")

            # Current value if available
            if adr is not None:
                value = manager.get_parameter(name)
                display_value = manager.get_parameter_display_value(name)

                if value is not None:
                    lines.append(f"  Current Value: {display_value}")
                    if str(value) != str(display_value):
                        lines.append(f"  Raw Value: {value}")
                else:
                    lines.append("  Current Value: <not read>")
            else:
                lines.append("  Current Value: <read-only/virtual>")

            # Description
            description = format_parameter_description(param_def)
            lines.append(f"  {description}")
            lines.append("")

    # Statistics
    lines.append(f"\n{'═' * 80}")
    lines.append("STATISTICS")
    lines.append(f"{'═' * 80}\n")

    total_params = len(PARAMETERS)
    readable_params = len([p for p in PARAMETERS if p.get('adr') is not None])
    current_params = len(manager.get_all_parameters())

    lines.append(f"Total parameters defined: {total_params}")
    lines.append(f"Readable parameters: {readable_params}")
    lines.append(f"Parameters read from gimbal: {current_params}")
    lines.append("")

    # Type breakdown
    type_counts = {}
    for param_def in PARAMETERS:
        ptype = param_def.get('type', 'UNKNOWN')
        type_counts[ptype] = type_counts.get(ptype, 0) + 1

    lines.append("Parameter types:")
    for ptype, count in sorted(type_counts.items()):
        lines.append(f"  {ptype:20s}: {count}")

    lines.append("\n" + "=" * 80)

    # Output
    output_text = '\n'.join(lines)

    if output_file:
        with open(output_file, 'w') as f:
            f.write(output_text)
        print(f"Parameter dump saved to {output_file}")
    else:
        print(output_text)


def dump_parameters_markdown(manager: ParameterManager, output_file: Path):
    """Dump parameters in Markdown table format."""

    lines = []
    lines.append("# SToRM32 Parameter Dump\n")

    # Group by page
    pages = {}
    for param_def in PARAMETERS:
        page = param_def.get('page', 'other')
        if page not in pages:
            pages[page] = []
        pages[page].append(param_def)

    for page_name, params in sorted(pages.items()):
        lines.append(f"\n## {page_name.upper()}\n")
        lines.append("| Parameter | Value | Type | Range/Choices | Default | Unit | Address |")
        lines.append("|-----------|-------|------|---------------|---------|------|---------|")

        for param_def in params:
            name = param_def.get('name', 'UNKNOWN')
            adr = param_def.get('adr', '-')
            param_type = param_def.get('type', 'UNKNOWN')

            # Current value
            value = manager.get_parameter_display_value(name) or '-'

            # Range or choices
            if param_type == 'LIST':
                choices = param_def.get('choices', [])
                if len(choices) <= 3:
                    range_str = ', '.join(choices)
                else:
                    range_str = f"{len(choices)} options"
            elif param_type in ['UINT', 'INT']:
                min_val = param_def.get('min', 0)
                max_val = param_def.get('max', 0)
                ppos = param_def.get('ppos', 0)
                if ppos > 0:
                    divisor = 10 ** ppos
                    min_val = min_val / divisor
                    max_val = max_val / divisor
                range_str = f"{min_val}-{max_val}"
            else:
                range_str = '-'

            # Default
            default = param_def.get('default', '-')
            if default != '-':
                ppos = param_def.get('ppos', 0)
                if ppos > 0:
                    default = default / (10 ** ppos)

            # Unit
            unit = param_def.get('unit', '-')

            lines.append(f"| {name} | {value} | {param_type} | {range_str} | {default} | {unit} | {adr} |")

        lines.append("")

    # Write to file
    with open(output_file, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Markdown parameter dump saved to {output_file}")


def dump_parameters_csv(manager: ParameterManager, output_file: Path):
    """Dump parameters in CSV format."""

    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)

        # Header
        writer.writerow([
            'Name', 'Page', 'Type', 'Address',
            'Current Value', 'Raw Value',
            'Min', 'Max', 'Default', 'Unit',
            'Choices', 'Description'
        ])

        # Data rows
        for param_def in sorted(PARAMETERS, key=lambda p: (p.get('page', 'zzz'), p.get('name', ''))):
            name = param_def.get('name', 'UNKNOWN')
            page = param_def.get('page', '')
            param_type = param_def.get('type', 'UNKNOWN')
            adr = param_def.get('adr', '')

            # Current values
            display_value = manager.get_parameter_display_value(name) or ''
            raw_value = manager.get_parameter(name) or ''

            # Metadata
            min_val = param_def.get('min', '')
            max_val = param_def.get('max', '')
            default = param_def.get('default', '')
            unit = param_def.get('unit', '')

            # Choices
            choices = param_def.get('choices', [])
            choices_str = '; '.join(choices) if choices else ''

            # Description
            description = format_parameter_description(param_def)

            writer.writerow([
                name, page, param_type, adr,
                display_value, raw_value,
                min_val, max_val, default, unit,
                choices_str, description
            ])

    print(f"CSV parameter dump saved to {output_file}")


def dump_parameters_json(manager: ParameterManager, output_file: Path):
    """Dump parameters in JSON format."""

    data = {
        'parameters': [],
        'statistics': {
            'total': len(PARAMETERS),
            'readable': len([p for p in PARAMETERS if p.get('adr') is not None]),
            'read_from_gimbal': len(manager.get_all_parameters())
        }
    }

    for param_def in PARAMETERS:
        param_data = {
            'name': param_def.get('name'),
            'type': param_def.get('type'),
            'page': param_def.get('page'),
            'address': param_def.get('adr'),
        }

        # Current values
        name = param_def.get('name')
        if name:
            raw_value = manager.get_parameter(name)
            display_value = manager.get_parameter_display_value(name)

            if raw_value is not None:
                param_data['current_value'] = display_value
                param_data['raw_value'] = raw_value

        # Metadata
        if 'min' in param_def:
            param_data['min'] = param_def['min']
        if 'max' in param_def:
            param_data['max'] = param_def['max']
        if 'default' in param_def:
            param_data['default'] = param_def['default']
        if 'unit' in param_def:
            param_data['unit'] = param_def['unit']
        if 'choices' in param_def:
            param_data['choices'] = param_def['choices']
        if 'ppos' in param_def:
            param_data['decimal_places'] = param_def['ppos']

        param_data['description'] = format_parameter_description(param_def)

        data['parameters'].append(param_data)

    # Write to file
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"JSON parameter dump saved to {output_file}")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Dump all SToRM32 gimbal parameters with descriptions',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Display to console (text format)
  %(prog)s /dev/ttyUSB0

  # Save to text file
  %(prog)s /dev/ttyUSB0 -o params.txt

  # Save as Markdown
  %(prog)s /dev/ttyUSB0 -o params.md -f markdown

  # Save as CSV
  %(prog)s /dev/ttyUSB0 -o params.csv -f csv

  # Save as JSON
  %(prog)s /dev/ttyUSB0 -o params.json -f json

  # Use default values (no gimbal connection required)
  %(prog)s --defaults -o params.txt
        """
    )

    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('-o', '--output', type=Path,
                        help='Output file (default: stdout for text format)')
    parser.add_argument('-f', '--format', choices=['text', 'markdown', 'csv', 'json'],
                        default='text',
                        help='Output format (default: text)')
    parser.add_argument('-d', '--defaults', action='store_true',
                        help='Use default values instead of reading from gimbal')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose logging')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.INFO)

    print("SToRM32 Parameter Dump Utility")
    print("=" * 60)

    # Create parameter manager
    manager = ParameterManager()

    # Read from gimbal or use defaults
    if args.defaults:
        print("Using default parameter values...")
        # Populate with defaults
        for param_def in PARAMETERS:
            adr = param_def.get('adr')
            default = param_def.get('default', 0)
            if adr is not None:
                manager._parameters[adr] = default
        print(f"Loaded {len(manager._parameters)} default values")
    else:
        print(f"Connecting to gimbal on {args.port}...")
        try:
            protocol = SerialProtocol(args.port)
            protocol.connect()
            print("Connected successfully")

            manager.protocol = protocol

            print("Reading parameters from gimbal...")
            protocol.set_extended_timeout(True)

            try:
                params = manager.read_from_gimbal()
                print(f"Read {len(params)} parameters from gimbal")
            except Exception as e:
                print(f"Warning: Failed to read parameters: {e}")
                print("Using default values instead...")
                # Populate with defaults
                for param_def in PARAMETERS:
                    adr = param_def.get('adr')
                    default = param_def.get('default', 0)
                    if adr is not None:
                        manager._parameters[adr] = default

            protocol.disconnect()

        except Exception as e:
            print(f"Error: Failed to connect to gimbal: {e}")
            print("Use --defaults to dump parameter definitions without hardware")
            return 1

    print(f"Generating {args.format} output...")
    print()

    # Generate output
    if args.format == 'text':
        dump_parameters_text(manager, args.output)
    elif args.format == 'markdown':
        if not args.output:
            args.output = Path('parameters.md')
        dump_parameters_markdown(manager, args.output)
    elif args.format == 'csv':
        if not args.output:
            args.output = Path('parameters.csv')
        dump_parameters_csv(manager, args.output)
    elif args.format == 'json':
        if not args.output:
            args.output = Path('parameters.json')
        dump_parameters_json(manager, args.output)

    return 0


if __name__ == '__main__':
    sys.exit(main())
