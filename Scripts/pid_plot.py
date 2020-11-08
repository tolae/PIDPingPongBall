import re
import atexit
import serial
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import time

input_reg = re.compile(r"(?P<target>-?\d+), (?P<measured>-?\d+), (?P<actual>-?\d+)")

class Data(object):
	def __init__(self, kp, ki, kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.target = []
		self.measured = []
		self.actual = []

def _parse_args():
	parser = argparse.ArgumentParser(
		"Continually grabs data, analyzes and plots it."
	)
	parser.add_argument('-p', '--port',
		required=True,
		help="Port connection."
	)
	parser.add_argument('-b', '--baud_rate',
		required=False,
		default=115200,
		help="Baud rate to open port connection to."
	)

	args = parser.parse_args()

	return args

def animate(frame, *fargs):
	ser = fargs[0]
	reads = fargs[1]
	pid_data = fargs[2]

	if ser.in_waiting >= 19:
		input_data = ser.read_all()
		read_data = input_data.decode('utf-8')
		matches = input_reg.findall(read_data)
		for match in matches:
			if match is not None:
				target, measured, actual = match
				if len(pid_data.target) > 100:
					pid_data.target.pop(0)
				if len(pid_data.measured) > 100:
					pid_data.measured.pop(0)
				if len(pid_data.actual) > 100:
					pid_data.actual.pop(0)
				pid_data.target.append(target)
				pid_data.measured.append(measured)
				pid_data.actual.append(actual)
				if len(reads) == 0:
					reads.append(1)
				else:
					if len(reads) <= 100:
						reads.append(reads[-1] + 1)

	ax1 = plt.subplot(1, 2, 1)
	ax1.clear()
	ax1.plot(reads, pid_data.target, 'o')
	# ax2 = plt.subplot(1, 3, 2)
	# ax2.clear()
	# ax2.plot(reads, pid_data.measured, 'o')
	ax3 = plt.subplot(1, 2, 2)
	ax3.clear()
	ax3.plot(reads, pid_data.actual, 'o')

def main():
	args = _parse_args()
	# Get serial connection
	ser = serial.Serial(
		port = args.port,
		baudrate = args.baud_rate,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		bytesize = serial.EIGHTBITS,
		timeout = 0
	)
	atexit.register(ser.close)
	reads = []

	# Clears any old cached data
	ser.reset_input_buffer()
	ser.flushInput()
	ser.read_all()

	pid_data = Data(0, 0, 0)

	time.sleep(2)

	ser.write("start".encode("ascii"))

	# Configure matplotlib
	style.use('fivethirtyeight')
	fig = plt.figure()
	an1 = animation.FuncAnimation(fig, animate, fargs=(ser, reads, pid_data), interval=25)
	plt.show()

if __name__ == "__main__":
	main()
