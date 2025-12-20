"""
Parameter validation utilities for SToRM32 configuration.
"""

from typing import Any, Optional, Tuple


class ValidationError(Exception):
    """Exception raised when parameter validation fails."""
    pass


def validate_uint(value: Any, min_val: int, max_val: int, param_name: str = "Parameter") -> int:
    """
    Validate unsigned integer parameter.

    Args:
        value: Value to validate (will be converted to int)
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        param_name: Parameter name for error messages

    Returns:
        Validated integer value

    Raises:
        ValidationError: If value is out of range or invalid
    """
    try:
        int_value = int(value)
    except (ValueError, TypeError):
        raise ValidationError(f"{param_name}: Invalid integer value '{value}'")

    if int_value < min_val or int_value > max_val:
        raise ValidationError(
            f"{param_name}: Value {int_value} out of range [{min_val}, {max_val}]"
        )

    return int_value


def validate_int(value: Any, min_val: int, max_val: int, param_name: str = "Parameter") -> int:
    """
    Validate signed integer parameter.

    Args:
        value: Value to validate (will be converted to int)
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        param_name: Parameter name for error messages

    Returns:
        Validated integer value

    Raises:
        ValidationError: If value is out of range or invalid
    """
    return validate_uint(value, min_val, max_val, param_name)


def validate_list_value(value: Any, valid_values: list, param_name: str = "Parameter") -> int:
    """
    Validate that value is in list of valid options.

    Args:
        value: Value to validate (integer index into list)
        valid_values: List of valid values/options
        param_name: Parameter name for error messages

    Returns:
        Validated index value

    Raises:
        ValidationError: If value is not in valid range
    """
    try:
        int_value = int(value)
    except (ValueError, TypeError):
        raise ValidationError(f"{param_name}: Invalid value '{value}'")

    if int_value < 0 or int_value >= len(valid_values):
        raise ValidationError(
            f"{param_name}: Value {int_value} not in valid range [0, {len(valid_values)-1}]"
        )

    return int_value


def validate_string(value: Any, max_length: Optional[int] = None, param_name: str = "Parameter") -> str:
    """
    Validate string parameter.

    Args:
        value: Value to validate (will be converted to string)
        max_length: Maximum string length (None for no limit)
        param_name: Parameter name for error messages

    Returns:
        Validated string value

    Raises:
        ValidationError: If string is too long
    """
    str_value = str(value)

    if max_length is not None and len(str_value) > max_length:
        raise ValidationError(
            f"{param_name}: String too long ({len(str_value)} > {max_length})"
        )

    return str_value


def validate_parameter(value: Any, param_def: dict) -> Tuple[Any, str]:
    """
    Validate parameter value against parameter definition.

    Args:
        value: Value to validate
        param_def: Parameter definition dictionary with keys:
            - type: 'UINT', 'INT', 'LIST', 'STR', 'STR+READONLY'
            - min, max: Range for UINT/INT
            - len: Max length for STR
            - valid_values: List for LIST type

    Returns:
        Tuple of (validated_value, error_message)
        error_message is empty string if validation successful

    Example:
        >>> param_def = {'name': 'Test', 'type': 'UINT', 'min': 0, 'max': 100}
        >>> validate_parameter(50, param_def)
        (50, '')
        >>> validate_parameter(150, param_def)
        (None, 'Test: Value 150 out of range [0, 100]')
    """
    param_name = param_def.get('name', 'Parameter')
    param_type = param_def.get('type', 'UINT')

    try:
        if param_type == 'UINT':
            validated = validate_uint(
                value,
                param_def.get('min', 0),
                param_def.get('max', 65535),
                param_name
            )
            return (validated, '')

        elif param_type == 'INT':
            validated = validate_int(
                value,
                param_def.get('min', -32768),
                param_def.get('max', 32767),
                param_name
            )
            return (validated, '')

        elif param_type == 'LIST':
            # For LIST type, check against min/max which represent index range
            validated = validate_uint(
                value,
                param_def.get('min', 0),
                param_def.get('max', 0),
                param_name
            )
            return (validated, '')

        elif param_type in ('STR', 'STR+READONLY'):
            max_len = param_def.get('len', None)
            validated = validate_string(value, max_len, param_name)
            return (validated, '')

        else:
            return (None, f"{param_name}: Unknown parameter type '{param_type}'")

    except ValidationError as e:
        return (None, str(e))


def clamp_value(value: int, min_val: int, max_val: int) -> int:
    """
    Clamp value to range.

    Args:
        value: Value to clamp
        min_val: Minimum value
        max_val: Maximum value

    Returns:
        Clamped value

    Example:
        >>> clamp_value(150, 0, 100)
        100
        >>> clamp_value(-10, 0, 100)
        0
        >>> clamp_value(50, 0, 100)
        50
    """
    return max(min_val, min(max_val, value))


if __name__ == '__main__':
    # Simple tests
    print("=== Validation Tests ===")

    # UINT validation
    try:
        result = validate_uint(50, 0, 100, "Test")
        print(f"validate_uint(50, 0, 100): {result} ✓")
    except ValidationError as e:
        print(f"Error: {e}")

    try:
        result = validate_uint(150, 0, 100, "Test")
        print(f"validate_uint(150, 0, 100): {result}")
    except ValidationError as e:
        print(f"Expected error: {e} ✓")

    # Parameter validation with definition
    param_def = {
        'name': 'Pitch P',
        'type': 'UINT',
        'min': 0,
        'max': 3000,
        'ppos': 2
    }

    value, error = validate_parameter(1500, param_def)
    print(f"\nvalidate_parameter(1500): value={value}, error='{error}' ✓")

    value, error = validate_parameter(5000, param_def)
    print(f"validate_parameter(5000): value={value}, error='{error}' ✓")

    # Clamp test
    print(f"\nclamp_value(150, 0, 100): {clamp_value(150, 0, 100)} ✓")
