import hub
from spikedev.utils.motor import SpikeSmallMotor

#config motors
motor = SpikeSmallMotor(hub.port.A)
motor = SpikeSmallMotor(hub.port.B)

#begin of motorpair driver
