# -*- coding: utf-8 -*-
import os
import select
import socket

import serial
from pyfirmata import ANALOG, DIGITAL, INPUT, OUTPUT, PWM, SERVO_CONFIG, Board, util

BOARDS = {
    "duinobot": {
        "digital": tuple(x for x in range(19)),
        "analog": tuple(x for x in range(7)),
        "pwm": (5, 6, 9),
        "use_ports": True,
        "disabled": (0, 1, 3, 4, 8),
    }
}

# extended command set using sysex (0-127/0x00-0x7F)
# 0x00-0x0F reserved for user-defined commands */
SYSEX_PING = 0x03
ANALOG_INPUT_REQUEST = 0x06
DIGITAL_INPUT_REQUEST = 0x07
BROADCAST_REPORT = 0x09
PIN_COMMANDS = 0x0F
PIN_GET_ANALOG = 0x01
PIN_GET_DIGITAL = 0x02


class FirmataSerialBoard(Board):
    nearest_obstacle = [-1 for i in range(128)]
    analog_value = [-1 for i in range(128)]
    digital_value = [-1 for i in range(128)]
    live_robots = [0 for i in range(128)]

    def __init__(
        self, port, layout=None, baudrate=57600, name=None, timeout=None, debug=False
    ):
        self._pin_analog_value = {}
        self._pin_digital_value = {}

        try:
            super().__init__(port, layout, baudrate, name, timeout)
        except serial.SerialException:
            if os.path.exists(port) and not os.access(port, os.R_OK | os.W_OK):
                print(
                    "No tiene permisos para acceder al dispositivo,"
                    " verifique si su usuario pertenece al grupo dialout."
                )
            else:
                print(
                    "No es posible conectarse al robot, por favor enchufe "
                    "y configure el XBee."
                )
            if debug:
                raise  # re-raise the exception to allow the caller to handle this
            else:
                exit(1)

    def setup_layout(self, board_layout):
        # Setup default handlers for standard incoming commands
        super().setup_layout(board_layout)
        self.add_cmd_handler(SYSEX_PING, self._handle_sysex_ping)
        self.add_cmd_handler(ANALOG_INPUT_REQUEST, self._handle_sysex_analog)
        self.add_cmd_handler(DIGITAL_INPUT_REQUEST, self._handle_sysex_digital)
        self.add_cmd_handler(BROADCAST_REPORT, self._handle_sysex_broadcast)
        self.add_cmd_handler(PIN_COMMANDS, self._handle_sysex_pin_commands)

    def _handle_sysex_ping(self, *data):
        most_significant = data[0]
        least_significant = data[1]
        robot = data[2]
        self.nearest_obstacle[robot] = (most_significant << 7) + least_significant

    def _handle_sysex_analog(self, *data):
        most_significant = data[0]
        least_significant = data[1]
        robot = data[2]
        self.analog_value[robot] = (most_significant << 7) + least_significant

    def _handle_sysex_digital(self, *data):
        value = data[0]
        robot = data[1]
        self.digital_value[robot] = value

    def _handle_sysex_broadcast(self, *data):
        robot = data[0]
        self.live_robots[robot] = 1

    def pin_analog_value(self, robotid):
        return self._pin_analog_value.setdefault(robotid, [-1] * len(self.analog))

    def pin_digital_value(self, robotid):
        return self._pin_digital_value.setdefault(robotid, [-1] * len(self.digital))

    def _handle_sysex_pin_commands(self, *data):
        # Versión mejorada que reemplaza a sysex_analog y sysex_digital
        if data[0] == PIN_GET_ANALOG:
            most_significant = data[1]
            least_significant = data[2]
            pin = data[3]
            robot = data[4]
            self.pin_analog_value(robot)[pin] = (
                most_significant << 7
            ) + least_significant
        elif data[0] == PIN_GET_DIGITAL:
            value = data[1]
            pin = data[2]
            robot = data[3]
            self.pin_digital_value(robot)[pin] = value


class _WrapTCPSocket(object):
    def __init__(self, socket):
        self.skt = socket

    def read(self):
        try:
            return self.skt.recv(1)
        except socket.error:
            return ""

    def write(self, data):
        self.skt.sendall(data)

    def inWaiting(self):
        inputready, _, _ = select.select([self.skt.fileno()], [], [], 0)
        return 1 if inputready else 0

    def close(self):
        self.skt.close()


class FirmataTCPBoard(FirmataSerialBoard):
    def __init__(self, robot_ip, port, layout, name=None, debug=False):
        self._pin_analog_value = {}
        self._pin_digital_value = {}
        try:
            # Se crea el socket
            self.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.skt.settimeout(5)
            # Se crea la conexión
            self.skt.connect((robot_ip, port))
        except socket.error:
            print(
                "Error al intentar conectarse al robot, verifique "
                "que el módulo WiFi esté encendido."
            )
            if debug:
                raise
            else:
                exit(1)
        # Wrapper que simula una instancia de Serial
        self.sp = _WrapTCPSocket(self.skt)
        self.ip = robot_ip
        self.port = port
        if name is None:
            self.name = "{}:{}".format(robot_ip, port)
        else:
            self.name = name
        self._generic_initialization(layout)

    def _generic_initialization(self, layout):
        self._layout = layout
        if layout:
            self.setup_layout(layout)
        else:
            self.auto_setup()

        # Iterate over the first messages to get firmware data
        while self.bytes_available():
            self.iterate()
        # TODO Test whether we got a firmware name and version, otherwise there
        # probably isn't any Firmata installed
