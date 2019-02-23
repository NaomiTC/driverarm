# Uses the pySerial library to send and receive data from a
# Simple Motor Controller G2.
#
# NOTE: The Simple Motor Controller's input mode must be "Serial/USB".
# NOTE: You might need to change the "port_name =" line below to specify the
#   right serial port.

import serial


class SmcG2Serial(object):
	def __init__(self, port):
		self.port = port
		self.final_speed = 0

	def send_command(self, device, cmd, *data_bytes):
		header = [0xAA, device, cmd & 0x7F]  # Pololu protocol
		self.port.write(header + list(data_bytes))

	# Sends the Exit Safe Start command, which is required to drive the motor.
	def exit_safe_start(self,device):
		self.send_command(device,0x83)

	# Sets the SMC's target speed (-3200 to 3200).
	def set_target_speed(self, speed, device):
		cmd = 0x85  # Motor forward
		if speed < 0:
	  		cmd = 0x86  # Motor reverse
	 		speed = -speed
		self.send_command(device, cmd, int(speed) & 0x1F, int(speed) >> 5 & 0x7F)

	# Gets the specified variable as an unsigned value.
	def get_variable(self, id):
		self.send_command(0xA1, id)
		result = self.port.read(2)
		if len(result) != 2:
	 		raise RuntimeError("Expected to read 2 bytes, got {}."
		.format(len(result)))
			b = bytearray(result)
		return b[0] + 256 * b[1]

	# Gets the specified variable as a signed value.
	def get_variable_signed(self, id):
		value = self.get_variable(id)
		if value >= 0x8000:
	 		value -= 0x10000
		return value

	# Gets the target speed (-3200 to 3200).
	def get_target_speed(self):
		return self.get_variable_signed(20)

	# Gets a number where each bit represents a different error, and the
	# bit is 1 if the error is currently active.
	# See the user's guide for definitions of the different error bits.
	def get_error_status(self):
		return self.get_variable(0)

	#Define the direction of the motor on the same driver
	def select_direction(self, forward, backward, speed, init_val_backward, array_pos):
		if forward != 0 or backward != init_val_backward:
			if forward != 0:
				self.final_speed = -speed 
			elif backward != init_val_backward:
				self.final_speed = speed 
		else:
			self.final_speed = 0

		return self.final_speed


