import hub
from spikedev.utils.motor import MotorSpeedRPS, SpikeSmallMotor, SpikeMeduimMotor

# configure motor and sensors
motor = SpikeSmallMotor (hub.port.B)


# run for 20 degrees at 1.5 rotations-per-second
motor.run_for_degrees(720, MotorSpeedRPS(1.5))


