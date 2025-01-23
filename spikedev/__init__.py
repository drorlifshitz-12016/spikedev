import hub
from spikedev.motor import MotorSpeedRPS, SpikeMediumMotor

# run for 20 degrees at 1.5 rotations-per-second
motor = SpikeMediumMotor(hub.port.E)
motor.run_for_degrees(720, MotorSpeedRPS(1.5))
