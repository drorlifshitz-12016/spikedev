import hub

from spikedev.utils.motor import Motor_pair, MoveSteering, MotorSpeedDPS

Motor_pair = MoveSteering (hub.port.A , hub.port.B)

Motor_pair.run_for_time(3000, MotorSpeedDPS(180) MotorSpeedDPS(360))

