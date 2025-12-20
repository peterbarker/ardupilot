#!/usr/bin/env python3
"""
Parse SToRM32 parameter definitions from Perl source code.

Extracts parameter definitions from the Perl OptionList array and converts
them to Python data structures for use in the Python configuration tool.
"""

import re
import json
from pathlib import Path


def parse_perl_parameters(perl_file_path):
    """Parse parameter definitions from Perl file."""
    with open(perl_file_path, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()

    # Find the OptionList_L236 array
    match = re.search(r'my @OptionList_L236 = \((.*?)\n\);', content, re.DOTALL)
    if not match:
        raise ValueError("Could not find OptionList_L236 in Perl file")

    option_list_content = match.group(1)

    # Split into individual parameter definitions
    # Each definition starts with {, possibly with comments after },{
    # Match },{optional comment and whitespace}{
    param_blocks = re.split(r'\},?\s*(?:#[^\n]*)?\s*\{', option_list_content)

    parameters = []

    for block in param_blocks:
        # Clean up the block
        block = block.strip()
        if not block:
            continue

        # Remove leading { and trailing }
        block = re.sub(r'^\s*\{', '', block)
        block = re.sub(r'\}\s*,?\s*$', '', block)

        # Parse key => value pairs
        param = {}

        # Extract name
        name_match = re.search(r"name\s*=>\s*'([^']*)'", block)
        if name_match:
            param['name'] = name_match.group(1)
        else:
            # Try double quotes
            name_match = re.search(r'name\s*=>\s*"([^"]*)"', block)
            if name_match:
                param['name'] = name_match.group(1)
            else:
                continue  # Skip if no name

        # Extract type
        type_match = re.search(r"type\s*=>\s*'([^']*)'", block)
        if type_match:
            param['type'] = type_match.group(1)

        # Extract numeric fields
        for field in ['len', 'ppos', 'min', 'max', 'default', 'steps', 'adr', 'size']:
            field_match = re.search(rf'{field}\s*=>\s*(-?\d+)', block)
            if field_match:
                param[field] = int(field_match.group(1))

        # Extract unit
        unit_match = re.search(r"unit\s*=>\s*'([^']*)'", block)
        if unit_match:
            param['unit'] = unit_match.group(1)

        # Extract page
        page_match = re.search(r"page\s*=>\s*'([^']*)'", block)
        if page_match:
            param['page'] = page_match.group(1)

        # Extract choices array for LIST type
        choices_match = re.search(r"choices\s*=>\s*\[(.*?)\]", block, re.DOTALL)
        if choices_match:
            choices_str = choices_match.group(1)
            # Extract quoted strings
            choices = re.findall(r"'([^']*)'", choices_str)
            param['choices'] = choices

        # Extract pos array
        pos_match = re.search(r'pos\s*=>\s*\[(\d+),\s*(\d+)\]', block)
        if pos_match:
            param['pos'] = [int(pos_match.group(1)), int(pos_match.group(2))]

        # Extract column
        column_match = re.search(r'column\s*=>\s*(\d+)', block)
        if column_match:
            param['column'] = int(column_match.group(1))

        parameters.append(param)

    return parameters


def generate_python_code(parameters):
    """Generate Python code for parameter definitions."""
    code = '''"""
SToRM32 Parameter Definitions

Auto-generated from Perl source code.
Contains definitions for all 155+ gimbal parameters.
"""

from typing import List, Dict, Any, Optional


# Parameter type constants
PARAM_TYPE_UINT = 'UINT'
PARAM_TYPE_INT = 'INT'
PARAM_TYPE_LIST = 'LIST'
PARAM_TYPE_STR = 'STR'
PARAM_TYPE_STR_READONLY = 'STR+READONLY'


class ParameterDefinition:
    """Definition of a SToRM32 parameter."""

    def __init__(self, name: str, param_type: str, **kwargs):
        """
        Initialize parameter definition.

        Args:
            name: Parameter name
            param_type: Parameter type (UINT, INT, LIST, STR, STR+READONLY)
            **kwargs: Additional parameter attributes:
                len: Display length
                ppos: Decimal position (number of decimal places)
                min: Minimum value
                max: Maximum value
                default: Default value
                steps: Step size for UI
                adr: Parameter address/index
                size: Size for string parameters
                unit: Unit string
                page: GUI page/tab name
                pos: [row, column] position on page
                column: Column number
                choices: List of string choices for LIST type
        """
        self.name = name
        self.type = param_type
        self.len = kwargs.get('len', 0)
        self.ppos = kwargs.get('ppos', 0)
        self.min = kwargs.get('min', 0)
        self.max = kwargs.get('max', 0)
        self.default = kwargs.get('default', 0)
        self.steps = kwargs.get('steps', 1)
        self.adr = kwargs.get('adr', None)
        self.size = kwargs.get('size', 0)
        self.unit = kwargs.get('unit', '')
        self.page = kwargs.get('page', '')
        self.pos = kwargs.get('pos', None)
        self.column = kwargs.get('column', None)
        self.choices = kwargs.get('choices', [])

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        result = {
            'name': self.name,
            'type': self.type,
        }

        # Only include non-default values
        if self.len:
            result['len'] = self.len
        if self.ppos:
            result['ppos'] = self.ppos
        if self.min or self.min == 0:
            result['min'] = self.min
        if self.max:
            result['max'] = self.max
        if self.default or self.default == 0:
            result['default'] = self.default
        if self.steps != 1:
            result['steps'] = self.steps
        if self.adr is not None:
            result['adr'] = self.adr
        if self.size:
            result['size'] = self.size
        if self.unit:
            result['unit'] = self.unit
        if self.page:
            result['page'] = self.page
        if self.pos:
            result['pos'] = self.pos
        if self.column:
            result['column'] = self.column
        if self.choices:
            result['choices'] = self.choices

        return result


# Parameter definitions
PARAMETERS: List[Dict[str, Any]] = [
'''

    # Add each parameter as a dictionary
    for param in parameters:
        code += '    ' + json.dumps(param, indent=4).replace('\n', '\n    ') + ',\n'

    code += ''']


def get_parameter_by_name(name: str) -> Optional[Dict[str, Any]]:
    """Get parameter definition by name."""
    for param in PARAMETERS:
        if param.get('name') == name:
            return param
    return None


def get_parameter_by_address(adr: int) -> Optional[Dict[str, Any]]:
    """Get parameter definition by address."""
    for param in PARAMETERS:
        if param.get('adr') == adr:
            return param
    return None


def get_parameters_by_page(page: str) -> List[Dict[str, Any]]:
    """Get all parameters for a specific page."""
    return [p for p in PARAMETERS if p.get('page') == page]


def get_all_parameters() -> List[Dict[str, Any]]:
    """Get all parameter definitions."""
    return PARAMETERS.copy()
'''

    return code


def main():
    """Main entry point."""
    perl_file = Path('/tmp/storm32bgc/firmware binaries & gui/o323bgc-release-v240-v20180807/o323BGCTool_v240.pl')

    if not perl_file.exists():
        print(f"Error: Perl file not found at {perl_file}")
        return 1

    print(f"Parsing parameter definitions from {perl_file}...")
    parameters = parse_perl_parameters(perl_file)

    print(f"Found {len(parameters)} parameters")

    # Show summary
    param_types = {}
    params_with_adr = 0
    for param in parameters:
        param_type = param.get('type', 'UNKNOWN')
        param_types[param_type] = param_types.get(param_type, 0) + 1
        if 'adr' in param:
            params_with_adr += 1

    print("\nParameter types:")
    for ptype, count in sorted(param_types.items()):
        print(f"  {ptype}: {count}")

    print(f"\nParameters with address: {params_with_adr}")

    # Generate Python code
    print("\nGenerating Python code...")
    python_code = generate_python_code(parameters)

    output_file = Path(__file__).parent / 'models' / 'parameter_definitions.py'
    output_file.parent.mkdir(exist_ok=True)

    with open(output_file, 'w') as f:
        f.write(python_code)

    print(f"Wrote {len(python_code)} bytes to {output_file}")

    # Show first few parameters
    print("\nFirst 5 parameters:")
    for i, param in enumerate(parameters[:5]):
        print(f"{i}: {param.get('name')} ({param.get('type')})")
        if 'adr' in param:
            print(f"   Address: {param['adr']}")
        if 'default' in param:
            print(f"   Default: {param['default']}")

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
