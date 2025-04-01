import hub
from spikedev.utils.motor import MotorSpeedRPS, SpikeSmallMotor, SpikeMeduimMotor
from spikedev.utils.sensor import ColorSensor
from lib2to3.pgen2.tokenize import blank_re

# configure motor to port B
motor = SpikeSmallMotor (hub.port.B)
motor = SpikeMeduimMotor (hub.port.C)
sensor = ColorSensor (hub.port.A)

# run for 20 degrees at 1.5 rotations-per-second
motor.run_for_degrees(720, MotorSpeedRPS(1.5))

#color sensor identify black
sensor.color(blank_re)

