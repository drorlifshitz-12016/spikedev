# spikedev libraries
from spikedev import ButtonLeft
from spikedev import log_msg

log_msg("start")
btn = ButtonLeft()
btn.wait_for_pressed(5000)
log_msg("finish")
