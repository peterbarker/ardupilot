"""
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
    {
        "name": "Firmware Version",
        "type": "STR+READONLY",
        "size": 16,
        "page": "dashboard",
        "column": 1
    },
    {
        "name": "Board",
        "type": "STR+READONLY",
        "size": 16
    },
    {
        "name": "Name",
        "type": "STR+READONLY",
        "size": 16
    },
    {
        "name": "Gyro LPF",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 1,
        "steps": 1,
        "adr": 12,
        "page": "pid",
        "choices": [
            "off",
            "1.5 ms",
            "3.0 ms",
            "4.5 ms",
            "6.0 ms",
            "7.5 ms",
            "9 ms"
        ],
        "pos": [
            1,
            1
        ]
    },
    {
        "name": "Foc Gyro LPF",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 1,
        "steps": 1,
        "adr": 41,
        "choices": [
            "off",
            "1.5 ms",
            "3.0 ms",
            "4.5 ms",
            "6.0 ms",
            "7.5 ms",
            "9 ms"
        ],
        "pos": [
            1,
            1
        ]
    },
    {
        "name": "Imu2 FeedForward LPF",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 1,
        "steps": 1,
        "adr": 72,
        "choices": [
            "off",
            "1.5 ms",
            "4 ms",
            "10 ms",
            "22 ms",
            "46 ms",
            "94 ms"
        ]
    },
    {
        "name": "Voltage Correction",
        "type": "UINT",
        "len": 7,
        "ppos": 0,
        "min": 0,
        "max": 200,
        "default": 0,
        "steps": 1,
        "adr": 75,
        "unit": "%",
        "pos": [
            1,
            4
        ]
    },
    {
        "name": "Roll Yaw PD Mixing",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 100,
        "default": 0,
        "steps": 1,
        "adr": 73,
        "unit": "%",
        "pos": [
            4,
            6
        ]
    },
    {
        "name": "Pitch P",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 3000,
        "default": 400,
        "steps": 10,
        "adr": 0,
        "pos": [
            2,
            1
        ]
    },
    {
        "name": "Pitch I",
        "type": "UINT",
        "len": 7,
        "ppos": 1,
        "min": 0,
        "max": 32000,
        "default": 1000,
        "steps": 50,
        "adr": 1
    },
    {
        "name": "Pitch D",
        "type": "UINT",
        "len": 3,
        "ppos": 4,
        "min": 0,
        "max": 8000,
        "default": 500,
        "steps": 50,
        "adr": 2
    },
    {
        "name": "Pitch Motor Vmax",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 255,
        "default": 150,
        "steps": 1,
        "adr": 3
    },
    {
        "name": "Roll P",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 3000,
        "default": 400,
        "steps": 10,
        "adr": 4,
        "pos": [
            3,
            1
        ]
    },
    {
        "name": "Roll I",
        "type": "UINT",
        "len": 7,
        "ppos": 1,
        "min": 0,
        "max": 32000,
        "default": 1000,
        "steps": 50,
        "adr": 5
    },
    {
        "name": "Roll D",
        "type": "UINT",
        "len": 3,
        "ppos": 4,
        "min": 0,
        "max": 8000,
        "default": 500,
        "steps": 50,
        "adr": 6
    },
    {
        "name": "Roll Motor Vmax",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 255,
        "default": 150,
        "steps": 1,
        "adr": 7
    },
    {
        "name": "Yaw P",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 3000,
        "default": 400,
        "steps": 10,
        "adr": 8,
        "pos": [
            4,
            1
        ]
    },
    {
        "name": "Yaw I",
        "type": "UINT",
        "len": 7,
        "ppos": 1,
        "min": 0,
        "max": 32000,
        "default": 1000,
        "steps": 50,
        "adr": 9
    },
    {
        "name": "Yaw D",
        "type": "UINT",
        "len": 3,
        "ppos": 4,
        "min": 0,
        "max": 8000,
        "default": 500,
        "steps": 50,
        "adr": 10
    },
    {
        "name": "Yaw Motor Vmax",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 255,
        "default": 150,
        "steps": 1,
        "adr": 11
    },
    {
        "name": "Foc Pitch P",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 3000,
        "default": 400,
        "steps": 10,
        "adr": 23,
        "pos": [
            2,
            1
        ]
    },
    {
        "name": "Foc Pitch I",
        "type": "UINT",
        "len": 7,
        "ppos": 1,
        "min": 0,
        "max": 32000,
        "default": 100,
        "steps": 50,
        "adr": 24
    },
    {
        "name": "Foc Pitch D",
        "type": "UINT",
        "len": 3,
        "ppos": 4,
        "min": 0,
        "max": 8000,
        "default": 2000,
        "steps": 50,
        "adr": 25
    },
    {
        "name": "Foc Pitch K",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 1,
        "max": 100,
        "default": 10,
        "steps": 1,
        "adr": 26
    },
    {
        "name": "Foc Roll P",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 3000,
        "default": 400,
        "steps": 10,
        "adr": 29,
        "pos": [
            3,
            1
        ]
    },
    {
        "name": "Foc Roll I",
        "type": "UINT",
        "len": 7,
        "ppos": 1,
        "min": 0,
        "max": 32000,
        "default": 100,
        "steps": 50,
        "adr": 30
    },
    {
        "name": "Foc Roll D",
        "type": "UINT",
        "len": 3,
        "ppos": 4,
        "min": 0,
        "max": 8000,
        "default": 2000,
        "steps": 50,
        "adr": 31
    },
    {
        "name": "Foc Roll K",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 1,
        "max": 100,
        "default": 10,
        "steps": 1,
        "adr": 32
    },
    {
        "name": "Foc Yaw P",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 3000,
        "default": 400,
        "steps": 10,
        "adr": 35,
        "pos": [
            4,
            1
        ]
    },
    {
        "name": "Foc Yaw I",
        "type": "UINT",
        "len": 7,
        "ppos": 1,
        "min": 0,
        "max": 32000,
        "default": 100,
        "steps": 50,
        "adr": 36
    },
    {
        "name": "Foc Yaw D",
        "type": "UINT",
        "len": 3,
        "ppos": 4,
        "min": 0,
        "max": 8000,
        "default": 2000,
        "steps": 50,
        "adr": 37
    },
    {
        "name": "Foc Yaw K",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 1,
        "max": 100,
        "default": 10,
        "steps": 1,
        "adr": 38
    },
    {
        "name": "Pan Mode Control",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 79,
        "page": "pan",
        "column": 1
    },
    {
        "name": "Pan Mode Default Setting",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 5,
        "default": 0,
        "steps": 1,
        "adr": 80,
        "choices": [
            "hold hold pan",
            "hold hold hold",
            "pan pan pan",
            "pan hold hold",
            "pan hold pan",
            "hold pan pan",
            "off"
        ]
    },
    {
        "name": "Pan Mode Setting #1",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 1,
        "steps": 1,
        "adr": 81,
        "choices": [
            "hold hold pan",
            "hold hold hold",
            "pan pan pan",
            "pan hold hold",
            "pan hold pan",
            "hold pan pan",
            "off"
        ]
    },
    {
        "name": "Pan Mode Setting #2",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 4,
        "steps": 1,
        "adr": 82,
        "choices": [
            "hold hold pan",
            "hold hold hold",
            "pan pan pan",
            "pan hold hold",
            "pan hold pan",
            "hold pan pan",
            "off"
        ]
    },
    {
        "name": "Pan Mode Setting #3",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 2,
        "steps": 1,
        "adr": 83,
        "choices": [
            "hold hold pan",
            "hold hold hold",
            "pan pan pan",
            "pan hold hold",
            "pan hold pan",
            "hold pan pan",
            "off"
        ]
    },
    {
        "name": "Pitch Pan (0 = hold)",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 0,
        "max": 50,
        "default": 20,
        "steps": 1,
        "adr": 84,
        "column": 2
    },
    {
        "name": "Pitch Pan Deadband",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 0,
        "max": 600,
        "default": 0,
        "steps": 10,
        "adr": 85,
        "unit": "\ufffd",
        "pos": [
            2,
            3
        ]
    },
    {
        "name": "Pitch Pan Expo",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 100,
        "default": 0,
        "steps": 1,
        "adr": 86,
        "unit": "%"
    },
    {
        "name": "Roll Pan (0 = hold)",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 0,
        "max": 50,
        "default": 20,
        "steps": 1,
        "adr": 87,
        "column": 3
    },
    {
        "name": "Roll Pan Deadband",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 0,
        "max": 600,
        "default": 0,
        "steps": 10,
        "adr": 88,
        "unit": "\ufffd",
        "pos": [
            3,
            3
        ]
    },
    {
        "name": "Roll Pan Expo",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 100,
        "default": 0,
        "steps": 1,
        "adr": 89,
        "unit": "%"
    },
    {
        "name": "Yaw Pan (0 = hold)",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 0,
        "max": 50,
        "default": 20,
        "steps": 1,
        "adr": 90,
        "column": 4
    },
    {
        "name": "Yaw Pan Deadband",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 0,
        "max": 100,
        "default": 50,
        "steps": 5,
        "adr": 91,
        "unit": "\ufffd",
        "pos": [
            4,
            3
        ]
    },
    {
        "name": "Yaw Pan Expo",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 100,
        "default": 0,
        "steps": 1,
        "adr": 92,
        "unit": "%"
    },
    {
        "name": "Yaw Pan Deadband LPF",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 400,
        "default": 150,
        "steps": 5,
        "adr": 93,
        "unit": "s"
    },
    {
        "name": "Yaw Pan Deadband Hysteresis",
        "type": "UINT",
        "len": 5,
        "ppos": 1,
        "min": 0,
        "max": 50,
        "default": 0,
        "steps": 1,
        "adr": 94,
        "unit": "\ufffd",
        "pos": [
            4,
            6
        ]
    },
    {
        "name": "Rc Dead Band",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 50,
        "default": 10,
        "steps": 1,
        "adr": 96,
        "unit": "us",
        "page": "rcinputs"
    },
    {
        "name": "Rc Hysteresis",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 50,
        "default": 5,
        "steps": 1,
        "adr": 97,
        "unit": "us"
    },
    {
        "name": "Rc Pitch Trim",
        "type": "INT",
        "len": 0,
        "ppos": 0,
        "min": -100,
        "max": 100,
        "default": 0,
        "steps": 1,
        "adr": 104,
        "unit": "us",
        "pos": [
            1,
            4
        ]
    },
    {
        "name": "Rc Roll Trim",
        "type": "INT",
        "len": 0,
        "ppos": 0,
        "min": -100,
        "max": 100,
        "default": 0,
        "steps": 1,
        "adr": 111,
        "unit": "us"
    },
    {
        "name": "Rc Yaw Trim",
        "type": "INT",
        "len": 0,
        "ppos": 0,
        "min": -100,
        "max": 100,
        "default": 0,
        "steps": 1,
        "adr": 118,
        "unit": "us"
    },
    {
        "name": "Rc Pitch",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 102,
        "column": 2
    },
    {
        "name": "Rc Pitch Mode",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 103,
        "choices": [
            "absolute",
            "relative",
            "absolute centered"
        ]
    },
    {
        "name": "Rc Pitch Min",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -1200,
        "max": 1200,
        "default": -250,
        "steps": 5,
        "adr": 105,
        "unit": "\ufffd"
    },
    {
        "name": "Rc Pitch Max",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -1200,
        "max": 1200,
        "default": 250,
        "steps": 5,
        "adr": 106,
        "unit": "\ufffd"
    },
    {
        "name": "Rc Pitch Speed Limit (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 1,
        "min": 0,
        "max": 1000,
        "default": 400,
        "steps": 5,
        "adr": 107,
        "unit": "\ufffd/s"
    },
    {
        "name": "Rc Pitch Accel Limit (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 3,
        "min": 0,
        "max": 1000,
        "default": 300,
        "steps": 10,
        "adr": 108
    },
    {
        "name": "Rc Roll",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 109,
        "column": 3
    },
    {
        "name": "Rc Roll Mode",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 110,
        "choices": [
            "absolute",
            "relative",
            "absolute centered"
        ]
    },
    {
        "name": "Rc Roll Min",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -450,
        "max": 450,
        "default": -250,
        "steps": 5,
        "adr": 112,
        "unit": "\ufffd"
    },
    {
        "name": "Rc Roll Max",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -450,
        "max": 450,
        "default": 250,
        "steps": 5,
        "adr": 113,
        "unit": "\ufffd"
    },
    {
        "name": "Rc Roll Speed Limit (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 1,
        "min": 0,
        "max": 1000,
        "default": 400,
        "steps": 5,
        "adr": 114,
        "unit": "\ufffd/s"
    },
    {
        "name": "Rc Roll Accel Limit (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 3,
        "min": 0,
        "max": 1000,
        "default": 300,
        "steps": 10,
        "adr": 115
    },
    {
        "name": "Rc Yaw",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 116,
        "column": 4
    },
    {
        "name": "Rc Yaw Mode",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 3,
        "default": 0,
        "steps": 1,
        "adr": 117,
        "choices": [
            "absolute",
            "relative",
            "absolute centered",
            "relative turn around"
        ]
    },
    {
        "name": "Rc Yaw Min",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -2700,
        "max": 2700,
        "default": -250,
        "steps": 10,
        "adr": 119,
        "unit": "\ufffd"
    },
    {
        "name": "Rc Yaw Max",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -2700,
        "max": 2700,
        "default": 250,
        "steps": 10,
        "adr": 120,
        "unit": "\ufffd"
    },
    {
        "name": "Rc Yaw Speed Limit (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 1,
        "min": 0,
        "max": 1000,
        "default": 400,
        "steps": 5,
        "adr": 121,
        "unit": "\ufffd/s"
    },
    {
        "name": "Rc Yaw Accel Limit (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 3,
        "min": 0,
        "max": 1000,
        "default": 300,
        "steps": 10,
        "adr": 122
    },
    {
        "name": "Standby",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 123,
        "page": "functions",
        "column": 1
    },
    {
        "name": "Re-center Camera",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 124,
        "pos": [
            1,
            3
        ]
    },
    {
        "name": "IR Camera Control",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 125,
        "column": 2
    },
    {
        "name": "Camera Model",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 5,
        "default": 0,
        "steps": 1,
        "adr": 126,
        "choices": [
            "Sony Nex",
            "Canon",
            "Panasonic",
            "Nikon",
            "Git2 Rc",
            "CAMremote"
        ]
    },
    {
        "name": "IR Camera Setting #1",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 127,
        "choices": [
            "shutter",
            "shutter delay",
            "video on/off"
        ]
    },
    {
        "name": "IR Camera Setting #2",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 3,
        "default": 2,
        "steps": 1,
        "adr": 128,
        "choices": [
            "shutter",
            "shutter delay",
            "video on/off",
            "off"
        ]
    },
    {
        "name": "Time Interval (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 1,
        "min": 0,
        "max": 150,
        "default": 0,
        "steps": 1,
        "adr": 129,
        "unit": "s"
    },
    {
        "name": "Pwm Out Control",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "adr": 130,
        "column": 3
    },
    {
        "name": "Pwm Out Mid",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 900,
        "max": 2100,
        "default": 1500,
        "steps": 1,
        "adr": 131,
        "unit": "us"
    },
    {
        "name": "Pwm Out Min",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 900,
        "max": 2100,
        "default": 1100,
        "steps": 10,
        "adr": 132,
        "unit": "us"
    },
    {
        "name": "Pwm Out Max",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 900,
        "max": 2100,
        "default": 1900,
        "steps": 10,
        "adr": 133,
        "unit": "us"
    },
    {
        "name": "Pwm Out Speed Limit (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1000,
        "default": 0,
        "steps": 5,
        "adr": 134,
        "unit": "us/s"
    },
    {
        "name": "Script1 Control",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "page": "scripts",
        "column": 1
    },
    {
        "name": "Script2 Control",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "column": 2
    },
    {
        "name": "Script3 Control",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "column": 3
    },
    {
        "name": "Script4 Control",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "default": 0,
        "steps": 1,
        "column": 4
    },
    {
        "name": "Scripts",
        "type": "SCRIPT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 0,
        "steps": 0
    },
    {
        "name": "Imu2 Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 53,
        "page": "setup",
        "choices": [
            "off",
            "full",
            "full xy"
        ]
    },
    {
        "name": "Startup Mode",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 140,
        "choices": [
            "normal",
            "fast"
        ],
        "pos": [
            1,
            4
        ]
    },
    {
        "name": "Startup Delay",
        "type": "UINT",
        "len": 0,
        "ppos": 1,
        "min": 0,
        "max": 250,
        "default": 0,
        "steps": 5,
        "adr": 141,
        "unit": "s"
    },
    {
        "name": "Imu AHRS",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 2500,
        "default": 1000,
        "steps": 100,
        "adr": 61,
        "unit": "s"
    },
    {
        "name": "Virtual Channel Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 11,
        "default": 0,
        "steps": 1,
        "adr": 77,
        "choices": [
            "off",
            "sum ppm 6",
            "sum ppm 7",
            "sum ppm 8",
            "sum ppm 10",
            "sum ppm 12",
            "spektrum 10 bit",
            "spektrum 11 bit",
            "sbus",
            "hott sumd",
            "srxl",
            "serial"
        ],
        "column": 2
    },
    {
        "name": "Pwm Out Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 78,
        "choices": [
            "off",
            "1520 us 55 Hz",
            "1520 us 250 Hz"
        ]
    },
    {
        "name": "Rc Pitch Offset",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -1200,
        "max": 1200,
        "default": 0,
        "steps": 5,
        "adr": 99,
        "unit": "\ufffd",
        "pos": [
            2,
            4
        ]
    },
    {
        "name": "Rc Roll Offset",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -1200,
        "max": 1200,
        "default": 0,
        "steps": 5,
        "adr": 100,
        "unit": "\ufffd"
    },
    {
        "name": "Rc Yaw Offset",
        "type": "INT",
        "len": 0,
        "ppos": 1,
        "min": -1200,
        "max": 1200,
        "default": 0,
        "steps": 5,
        "adr": 101,
        "unit": "\ufffd"
    },
    {
        "name": "Esp Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 147,
        "choices": [
            "off",
            "uart",
            "uart2"
        ],
        "pos": [
            3,
            1
        ]
    },
    {
        "name": "Uart1 Tx Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 48,
        "choices": [
            "off",
            "oled display"
        ]
    },
    {
        "name": "Low Voltage Limit",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 11,
        "default": 1,
        "steps": 1,
        "adr": 74,
        "choices": [
            "off",
            "2.5 V/cell",
            "2.6 V/cell",
            "2.7 V/cell",
            "2.8 V/cell",
            "2.9 V/cell",
            "3.0 V/cell",
            "3.1 V/cell",
            "3.2 V/cell",
            "3.3 V/cell",
            "3.4 V/cell",
            "3.5 V/cell"
        ],
        "pos": [
            3,
            4
        ]
    },
    {
        "name": "Beep with Motors",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 143,
        "choices": [
            "off",
            "basic",
            "all"
        ],
        "pos": [
            3,
            4
        ]
    },
    {
        "name": "NT Logging",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 7,
        "default": 0,
        "steps": 1,
        "adr": 142,
        "choices": [
            "off",
            "basic",
            "basic + pid",
            "basic + accgyro",
            "basic + accgyro_raw",
            "basic + pid + accgyro",
            "basic + pid + ag_raw",
            "full"
        ]
    },
    {
        "name": "Pitch Motor Usage",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 3,
        "default": 3,
        "steps": 1,
        "adr": 55,
        "choices": [
            "normal",
            "level",
            "startup pos",
            "disabled"
        ],
        "column": 4
    },
    {
        "name": "Roll Motor Usage",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 3,
        "default": 3,
        "steps": 1,
        "adr": 56,
        "choices": [
            "normal",
            "level",
            "startup pos",
            "disabled"
        ]
    },
    {
        "name": "Yaw Motor Usage",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 3,
        "default": 3,
        "steps": 1,
        "adr": 57,
        "choices": [
            "normal",
            "level",
            "startup pos",
            "disabled"
        ]
    },
    {
        "name": "Imu Orientation",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 23,
        "default": 0,
        "steps": 1,
        "adr": 51,
        "page": "gimbalconfig",
        "pos": [
            1,
            1
        ]
    },
    {
        "name": "Imu2 Orientation",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 23,
        "default": 0,
        "steps": 1,
        "adr": 54
    },
    {
        "name": "Pitch Motor Poles",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 8,
        "max": 42,
        "default": 14,
        "steps": 2,
        "adr": 13,
        "pos": [
            2,
            1
        ]
    },
    {
        "name": "Pitch Motor Direction",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 2,
        "steps": 1,
        "adr": 14,
        "choices": [
            "normal",
            "reversed",
            "auto"
        ]
    },
    {
        "name": "Pitch Startup Motor Pos",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 1008,
        "default": 504,
        "steps": 1,
        "adr": 15
    },
    {
        "name": "Foc Pitch Motor Direction",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 42,
        "choices": [
            "normal",
            "reversed",
            "auto"
        ],
        "pos": [
            2,
            1
        ]
    },
    {
        "name": "Foc Pitch Zero Pos",
        "type": "INT",
        "len": 5,
        "ppos": 0,
        "min": -16384,
        "max": 16383,
        "default": 0,
        "steps": 8,
        "adr": 43
    },
    {
        "name": "Pitch Offset",
        "type": "INT",
        "len": 5,
        "ppos": 2,
        "min": -300,
        "max": 300,
        "default": 0,
        "steps": 5,
        "adr": 58,
        "unit": "\ufffd",
        "pos": [
            2,
            4
        ]
    },
    {
        "name": "Roll Motor Poles",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 8,
        "max": 42,
        "default": 14,
        "steps": 2,
        "adr": 16,
        "pos": [
            3,
            1
        ]
    },
    {
        "name": "Roll Motor Direction",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 2,
        "steps": 1,
        "adr": 17,
        "choices": [
            "normal",
            "reversed",
            "auto"
        ]
    },
    {
        "name": "Roll Startup Motor Pos",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 1008,
        "default": 504,
        "steps": 1,
        "adr": 18
    },
    {
        "name": "Foc Roll Motor Direction",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 44,
        "choices": [
            "normal",
            "reversed",
            "auto"
        ],
        "pos": [
            3,
            1
        ]
    },
    {
        "name": "Foc Roll Zero Pos",
        "type": "INT",
        "len": 5,
        "ppos": 0,
        "min": -16384,
        "max": 16383,
        "default": 0,
        "steps": 8,
        "adr": 45
    },
    {
        "name": "Roll Offset",
        "type": "INT",
        "len": 5,
        "ppos": 2,
        "min": -300,
        "max": 300,
        "default": 0,
        "steps": 5,
        "adr": 59,
        "unit": "\ufffd",
        "pos": [
            3,
            4
        ]
    },
    {
        "name": "Yaw Motor Poles",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 8,
        "max": 42,
        "default": 14,
        "steps": 2,
        "adr": 19,
        "pos": [
            4,
            1
        ]
    },
    {
        "name": "Yaw Motor Direction",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 2,
        "steps": 1,
        "adr": 20,
        "choices": [
            "normal",
            "reversed",
            "auto"
        ]
    },
    {
        "name": "Yaw Startup Motor Pos",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 1008,
        "default": 504,
        "steps": 1,
        "adr": 21
    },
    {
        "name": "Foc Yaw Motor Direction",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 46,
        "choices": [
            "normal",
            "reversed",
            "auto"
        ],
        "pos": [
            4,
            1
        ]
    },
    {
        "name": "Foc Yaw Zero Pos",
        "type": "INT",
        "len": 5,
        "ppos": 0,
        "min": -16384,
        "max": 16383,
        "default": 0,
        "steps": 8,
        "adr": 47
    },
    {
        "name": "Yaw Offset",
        "type": "INT",
        "len": 5,
        "ppos": 2,
        "min": -300,
        "max": 300,
        "default": 0,
        "steps": 5,
        "adr": 60,
        "unit": "\ufffd",
        "pos": [
            4,
            4
        ]
    },
    {
        "name": "Motor Mapping",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 5,
        "default": 0,
        "steps": 1,
        "adr": 22,
        "page": "expert",
        "choices": [
            "M0=pitch , M1=roll",
            "M0=roll , M1=pitch",
            "roll yaw pitch",
            "yaw roll pitch",
            "pitch yaw roll",
            "yaw pitch roll"
        ],
        "column": 3
    },
    {
        "name": "Imu Mapping",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 52,
        "choices": [
            "1 = id1 , 2 = id2",
            "1 = id2 , 2 = id1"
        ]
    },
    {
        "name": "Lipo Cells",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 0,
        "steps": 1,
        "adr": 49,
        "choices": [
            "auto",
            "1 S",
            "2 S",
            "3 S",
            "4 S",
            "5 S",
            "6 S"
        ],
        "pos": [
            3,
            4
        ]
    },
    {
        "name": "Lipo Voltage per Cell",
        "type": "UINT",
        "len": 0,
        "ppos": 2,
        "min": 300,
        "max": 500,
        "default": 420,
        "steps": 5,
        "adr": 50,
        "unit": "V"
    },
    {
        "name": "ADC Calibration",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 1000,
        "max": 2000,
        "default": 1550,
        "steps": 10,
        "adr": 76
    },
    {
        "name": "Imu3 Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 5,
        "default": 0,
        "steps": 1,
        "adr": 135,
        "choices": [
            "off",
            "default",
            "2 = id2, 3 = onboard",
            "2 = onboard, 3 = id2",
            "2 = onboard, 3 = id3",
            "2 = onboard, 3 = off"
        ],
        "column": 4
    },
    {
        "name": "Imu3 Orientation",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 23,
        "default": 0,
        "steps": 1,
        "adr": 136
    },
    {
        "name": "Uart1 Rx Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 139,
        "choices": [
            "off",
            "gps target"
        ],
        "pos": [
            4,
            5
        ]
    },
    {
        "name": "Acc LPF",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 2,
        "steps": 1,
        "adr": 71,
        "choices": [
            "off",
            "1.5 ms",
            "4.5 ms",
            "12 ms",
            "25 ms",
            "50 ms",
            "100 ms"
        ],
        "column": 1
    },
    {
        "name": "Rc Adc LPF",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 6,
        "default": 0,
        "steps": 1,
        "adr": 98,
        "choices": [
            "off",
            "1.5 ms",
            "4.5 ms",
            "12 ms",
            "25 ms",
            "50 ms",
            "100 ms"
        ]
    },
    {
        "name": "Hold To Pan Transition Time",
        "type": "UINT",
        "len": 5,
        "ppos": 0,
        "min": 0,
        "max": 1000,
        "default": 250,
        "steps": 25,
        "adr": 95,
        "unit": "ms"
    },
    {
        "name": "Acc Compensation Method",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 1,
        "steps": 1,
        "adr": 65,
        "choices": [
            "standard",
            "advanced"
        ],
        "pos": [
            1,
            6
        ]
    },
    {
        "name": "Uart Baudrate",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 7,
        "default": 0,
        "steps": 1,
        "adr": 68,
        "choices": [
            "default",
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "230400",
            "460800"
        ],
        "pos": [
            2,
            1
        ]
    },
    {
        "name": "Uart2 Baudrate",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 7,
        "default": 0,
        "steps": 1,
        "adr": 69,
        "choices": [
            "default",
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "230400",
            "460800"
        ]
    },
    {
        "name": "Usb Baudrate",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 7,
        "default": 0,
        "steps": 1,
        "adr": 70,
        "choices": [
            "default",
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "230400",
            "460800"
        ]
    },
    {
        "name": "Imu Acc Threshold (0 = off)",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 100,
        "default": 25,
        "steps": 1,
        "adr": 64,
        "unit": "g",
        "column": 2
    },
    {
        "name": "Acc Noise Level",
        "type": "UINT",
        "len": 0,
        "ppos": 3,
        "min": 0,
        "max": 150,
        "default": 40,
        "steps": 1,
        "adr": 66,
        "unit": "g"
    },
    {
        "name": "Acc Threshold (0 = off)",
        "type": "UINT",
        "len": 0,
        "ppos": 2,
        "min": 0,
        "max": 100,
        "default": 50,
        "steps": 1,
        "adr": 67,
        "unit": "g"
    },
    {
        "name": "Acc Vertical Weight",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 100,
        "default": 25,
        "steps": 5,
        "adr": 68,
        "unit": "%"
    },
    {
        "name": "Acc Zentrifugal Correction",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 100,
        "default": 30,
        "steps": 5,
        "adr": 69,
        "unit": "%"
    },
    {
        "name": "Acc Recover Time",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1000,
        "default": 250,
        "steps": 5,
        "adr": 70,
        "unit": " ms"
    },
    {
        "name": "Mavlink Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 3,
        "default": 0,
        "steps": 1,
        "adr": 144,
        "page": "interfaces",
        "choices": [
            "no heartbeat",
            "emit heartbeat",
            "heartbeat + attitude",
            "h.b. + mountstatus"
        ],
        "pos": [
            1,
            1
        ]
    },
    {
        "name": "Mavlink ComPort",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 66,
        "choices": [
            "uart",
            "usb",
            "uart2"
        ]
    },
    {
        "name": "Mavlink System ID",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 255,
        "default": 71,
        "steps": 1,
        "adr": 145
    },
    {
        "name": "Mavlink Component ID",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 255,
        "default": 67,
        "steps": 1,
        "adr": 146
    },
    {
        "name": "AP Compatibility",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 1,
        "default": 0,
        "steps": 1,
        "adr": 67,
        "choices": [
            "off",
            "apcrap+v0.96"
        ]
    },
    {
        "name": "Can Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 148,
        "choices": [
            "off",
            "uavcan",
            "dji naza"
        ],
        "pos": [
            2,
            1
        ]
    },
    {
        "name": "Uavcan Node ID",
        "type": "UINT",
        "len": 0,
        "ppos": 0,
        "min": 11,
        "max": 124,
        "default": 71,
        "steps": 1,
        "adr": 149
    },
    {
        "name": "STorM32Link Configuration",
        "type": "LIST",
        "len": 0,
        "ppos": 0,
        "min": 0,
        "max": 2,
        "default": 0,
        "steps": 1,
        "adr": 137,
        "choices": [
            "off",
            "yaw drift comp.",
            "v1"
        ],
        "pos": [
            3,
            1
        ]
    },
    {
        "name": "STorM32Link Pitch Offset",
        "type": "INT",
        "len": 5,
        "ppos": 2,
        "min": -500,
        "max": 500,
        "default": 0,
        "steps": 1,
        "adr": 39,
        "unit": "\ufffd"
    },
    {
        "name": "STorM32Link Roll Offset",
        "type": "INT",
        "len": 5,
        "ppos": 2,
        "min": -500,
        "max": 500,
        "default": 0,
        "steps": 1,
        "adr": 40,
        "unit": "\ufffd"
    },
    {
        "name": "STorM32Link AHRS Factor",
        "type": "UINT",
        "len": 5,
        "ppos": 2,
        "min": 0,
        "max": 200,
        "default": 100,
        "steps": 1,
        "adr": 34,
        "unit": "%"
    },
]


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
