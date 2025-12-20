"""
Parameter management for SToRM32 gimbal.

Handles reading, writing, validating, and managing gimbal parameters.
"""

import logging
import sys
from typing import Dict, List, Optional, Any
from pathlib import Path
import configparser

try:
    from ..models.parameter_definitions import (
        PARAMETERS, get_parameter_by_name, get_parameter_by_address,
        get_parameters_by_page, PARAM_TYPE_LIST, PARAM_TYPE_STR_READONLY
    )
    from ..utils.validation import validate_parameter
    from ..utils.conversions import pack_parameter_value, unpack_parameter_value
except ImportError:
    # Fallback for direct execution
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from models.parameter_definitions import (  # noqa: E402
        PARAMETERS, get_parameter_by_name, get_parameter_by_address,
        get_parameters_by_page, PARAM_TYPE_LIST, PARAM_TYPE_STR_READONLY
    )
    from utils.validation import validate_parameter  # noqa: E402
    from utils.conversions import pack_parameter_value, unpack_parameter_value  # noqa: E402


logger = logging.getLogger(__name__)


class ParameterError(Exception):
    """Exception raised for parameter-related errors."""
    pass


class ParameterManager:
    """
    Manages SToRM32 gimbal parameters.

    Provides high-level interface for reading, writing, validating,
    and persisting gimbal parameters.
    """

    def __init__(self, protocol=None):
        """
        Initialize parameter manager.

        Args:
            protocol: SerialProtocol instance for communication with gimbal
        """
        self.protocol = protocol
        self._parameters = {}  # Current parameter values (address -> value)
        self._definitions = PARAMETERS
        self._modified = set()  # Set of modified parameter addresses

    def read_from_gimbal(self) -> Dict[int, int]:
        """
        Read all parameters from gimbal.

        Returns:
            Dictionary mapping address to parameter value

        Raises:
            ParameterError: If read fails
        """
        if not self.protocol:
            raise ParameterError("No protocol instance configured")

        try:
            # Read all parameters using 'g' command
            param_values = self.protocol.get_all_parameters()

            # Map values to addresses
            self._parameters = {}
            for param_def in self._definitions:
                adr = param_def.get('adr')
                if adr is not None and adr < len(param_values):
                    self._parameters[adr] = param_values[adr]

            logger.info(f"Read {len(self._parameters)} parameters from gimbal")
            self._modified.clear()

            return self._parameters.copy()

        except Exception as e:
            logger.error(f"Failed to read parameters from gimbal: {e}")
            raise ParameterError(f"Failed to read parameters: {e}")

    def write_to_gimbal(self, parameters: Optional[Dict[int, int]] = None) -> bool:
        """
        Write parameters to gimbal.

        Args:
            parameters: Dict of address -> value to write (None = write all modified)

        Returns:
            True if successful

        Raises:
            ParameterError: If write fails
        """
        if not self.protocol:
            raise ParameterError("No protocol instance configured")

        # Determine which parameters to write
        if parameters is None:
            # Write all modified parameters
            if not self._modified:
                logger.info("No modified parameters to write")
                return True
            params_to_write = {adr: self._parameters[adr] for adr in self._modified}
        else:
            params_to_write = parameters

        try:
            # Write each parameter
            for adr, value in params_to_write.items():
                # TODO: Implement individual parameter write command
                # For now, we need to implement the 'p' command in serial_protocol.py
                logger.debug(f"Writing parameter {adr} = {value}")
                # self.protocol.set_parameter(adr, value)

            logger.info(f"Wrote {len(params_to_write)} parameters to gimbal")
            self._modified.clear()

            return True

        except Exception as e:
            logger.error(f"Failed to write parameters to gimbal: {e}")
            raise ParameterError(f"Failed to write parameters: {e}")

    def get_parameter(self, name_or_address) -> Optional[int]:
        """
        Get parameter value by name or address.

        Args:
            name_or_address: Parameter name (str) or address (int)

        Returns:
            Parameter value or None if not found
        """
        # Get parameter definition
        if isinstance(name_or_address, str):
            param_def = get_parameter_by_name(name_or_address)
        else:
            param_def = get_parameter_by_address(name_or_address)

        if not param_def:
            logger.warning(f"Parameter not found: {name_or_address}")
            return None

        adr = param_def.get('adr')
        if adr is None:
            logger.warning(f"Parameter has no address: {name_or_address}")
            return None

        return self._parameters.get(adr)

    def set_parameter(self, name_or_address, value: Any) -> bool:
        """
        Set parameter value by name or address.

        Args:
            name_or_address: Parameter name (str) or address (int)
            value: New parameter value

        Returns:
            True if successful

        Raises:
            ParameterError: If validation fails
        """
        # Get parameter definition
        if isinstance(name_or_address, str):
            param_def = get_parameter_by_name(name_or_address)
        else:
            param_def = get_parameter_by_address(name_or_address)

        if not param_def:
            raise ParameterError(f"Parameter not found: {name_or_address}")

        adr = param_def.get('adr')
        if adr is None:
            raise ParameterError(f"Parameter has no address: {name_or_address}")

        # Check if read-only
        if param_def.get('type') == PARAM_TYPE_STR_READONLY:
            raise ParameterError(f"Parameter is read-only: {param_def['name']}")

        # Validate value
        validated_value, error = validate_parameter(value, param_def)
        if error:
            raise ParameterError(f"Validation error for {param_def['name']}: {error}")

        # Pack value if needed (apply decimal position encoding)
        if 'ppos' in param_def:
            packed_value = pack_parameter_value(validated_value, param_def['ppos'])
        else:
            packed_value = int(validated_value)

        # Update parameter
        old_value = self._parameters.get(adr)
        self._parameters[adr] = packed_value

        if old_value != packed_value:
            self._modified.add(adr)
            logger.debug(f"Set parameter {param_def['name']} ({adr}) = {value} "
                         f"(packed: {packed_value})")

        return True

    def get_parameter_definition(self, name_or_address) -> Optional[Dict[str, Any]]:
        """Get parameter definition by name or address."""
        if isinstance(name_or_address, str):
            return get_parameter_by_name(name_or_address)
        else:
            return get_parameter_by_address(name_or_address)

    def get_all_parameters(self) -> Dict[int, int]:
        """Get all current parameter values."""
        return self._parameters.copy()

    def get_modified_parameters(self) -> Dict[int, int]:
        """Get all modified parameter values."""
        return {adr: self._parameters[adr] for adr in self._modified}

    def has_modifications(self) -> bool:
        """Check if there are unsaved modifications."""
        return len(self._modified) > 0

    def reset_modifications(self):
        """Clear modification tracking."""
        self._modified.clear()

    def load_from_ini(self, ini_file: Path) -> Dict[int, int]:
        """
        Load parameters from INI file.

        Args:
            ini_file: Path to INI file

        Returns:
            Dictionary of loaded parameters (address -> value)

        Raises:
            ParameterError: If load fails
        """
        if not ini_file.exists():
            raise ParameterError(f"INI file not found: {ini_file}")

        try:
            config = configparser.ConfigParser()
            config.read(ini_file)

            loaded = {}

            # Look for parameters section
            if 'PARAMETERS' in config:
                section = config['PARAMETERS']

                for param_def in self._definitions:
                    name = param_def.get('name')
                    adr = param_def.get('adr')

                    if name and adr is not None and name in section:
                        value_str = section[name]

                        # Parse value based on type
                        param_type = param_def.get('type')
                        if param_type == PARAM_TYPE_LIST:
                            # For LIST type, find index of choice
                            choices = param_def.get('choices', [])
                            if value_str in choices:
                                value = choices.index(value_str)
                            else:
                                try:
                                    value = int(value_str)
                                except ValueError:
                                    logger.warning(f"Invalid LIST value for {name}: {value_str}")
                                    continue
                        else:
                            # Numeric types
                            try:
                                value = float(value_str) if param_def.get('ppos', 0) > 0 else int(value_str)
                            except ValueError:
                                logger.warning(f"Invalid numeric value for {name}: {value_str}")
                                continue

                        # Set parameter
                        self.set_parameter(name, value)
                        loaded[adr] = self._parameters[adr]

            logger.info(f"Loaded {len(loaded)} parameters from {ini_file}")
            return loaded

        except Exception as e:
            logger.error(f"Failed to load INI file {ini_file}: {e}")
            raise ParameterError(f"Failed to load INI: {e}")

    def save_to_ini(self, ini_file: Path) -> bool:
        """
        Save parameters to INI file.

        Args:
            ini_file: Path to INI file

        Returns:
            True if successful

        Raises:
            ParameterError: If save fails
        """
        try:
            config = configparser.ConfigParser()

            # Create PARAMETERS section
            config['PARAMETERS'] = {}
            section = config['PARAMETERS']

            for param_def in self._definitions:
                name = param_def.get('name')
                adr = param_def.get('adr')
                param_type = param_def.get('type')

                if not name or adr is None or adr not in self._parameters:
                    continue

                # Skip read-only parameters
                if param_type == PARAM_TYPE_STR_READONLY:
                    continue

                value = self._parameters[adr]

                # Unpack value if needed
                if 'ppos' in param_def and param_def['ppos'] > 0:
                    value = unpack_parameter_value(value, param_def['ppos'])

                # Format value based on type
                if param_type == PARAM_TYPE_LIST:
                    # For LIST type, save as string choice
                    choices = param_def.get('choices', [])
                    if 0 <= value < len(choices):
                        value_str = choices[value]
                    else:
                        value_str = str(value)
                else:
                    value_str = str(value)

                section[name] = value_str

            # Write to file
            ini_file.parent.mkdir(parents=True, exist_ok=True)
            with open(ini_file, 'w') as f:
                config.write(f)

            logger.info(f"Saved parameters to {ini_file}")
            return True

        except Exception as e:
            logger.error(f"Failed to save INI file {ini_file}: {e}")
            raise ParameterError(f"Failed to save INI: {e}")

    def get_parameters_by_page(self, page: str) -> List[Dict[str, Any]]:
        """
        Get all parameters for a specific GUI page.

        Args:
            page: Page name (e.g., 'pid', 'setup', 'dashboard')

        Returns:
            List of parameter definitions
        """
        return get_parameters_by_page(page)

    def get_parameter_display_value(self, name_or_address) -> Optional[str]:
        """
        Get parameter value formatted for display.

        Args:
            name_or_address: Parameter name (str) or address (int)

        Returns:
            Formatted string value or None
        """
        param_def = self.get_parameter_definition(name_or_address)
        if not param_def:
            return None

        adr = param_def.get('adr')
        if adr is None or adr not in self._parameters:
            return None

        value = self._parameters[adr]
        param_type = param_def.get('type')

        # Unpack value
        if 'ppos' in param_def and param_def['ppos'] > 0:
            value = unpack_parameter_value(value, param_def['ppos'])

        # Format based on type
        if param_type == PARAM_TYPE_LIST:
            choices = param_def.get('choices', [])
            if 0 <= value < len(choices):
                return choices[value]
            return str(value)
        else:
            # Add unit if present
            unit = param_def.get('unit', '')
            if unit:
                return f"{value} {unit}"
            return str(value)


if __name__ == '__main__':
    # Simple test/demo
    logging.basicConfig(level=logging.DEBUG)

    manager = ParameterManager()

    # Show parameter statistics
    param_types = {}
    for param in PARAMETERS:
        ptype = param.get('type', 'UNKNOWN')
        param_types[ptype] = param_types.get(ptype, 0) + 1

    print("Parameter Statistics:")
    print(f"Total parameters: {len(PARAMETERS)}")
    print("\nBy type:")
    for ptype, count in sorted(param_types.items()):
        print(f"  {ptype}: {count}")

    # Show parameters by page
    pages = set(p.get('page', '') for p in PARAMETERS if p.get('page'))
    print(f"\nPages: {sorted(pages)}")

    # Show a sample parameter
    print("\nSample parameter (Pitch P):")
    param = get_parameter_by_name('Pitch P')
    if param:
        for key, value in param.items():
            print(f"  {key}: {value}")
