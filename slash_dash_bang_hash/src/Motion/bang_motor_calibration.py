import struct
import time
import serial
import matplotlib.pyplot as plt

# ser = serial.Serial('COM11', 115200, timeout=None) #windows
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)

# Note: If you would like to port the readFloat and writeFloat functions to C++, simply use the functions provided
# in the PSoC implementation: https://github.com/embeddedprogrammer/psoc-pid-controller/blob/master/serial.c
def writeFloat(f):
	ser.write(struct.pack('>i', int(f*1000)))
def readFloat():
	return float(struct.unpack('>i', ser.read(4))[0])/1000
def setPower(p1, p2, p3):
	ser.write('p')
	writeFloat(p1)
	writeFloat(p2)
	writeFloat(p3)
def enableLookupTable(enable):
	ser.write('n')
	writeFloat(enable)
def storeLookupValue(motor, pwm, speed, index): #use motor = 0 to set all motors
	ser.write('l')
	ser.write(str(motor))
	writeFloat(pwm)
	writeFloat(speed)
	writeFloat(index)
def setT(period_ms, tau_ms):
	ser.write('t')
	writeFloat(period_ms)
	writeFloat(tau_ms)
def getSpeed():
	ser.write('v')
	return readFloat(), readFloat(), readFloat()
def getEncoderCount():
	ser.write('e')
	return readFloat(), readFloat(), readFloat()
def disengage():
	ser.write('d')

# Set tick period (triggers PID control) and velocity filter corner frequency
setT(20, 100)

speed_settle_wait_time = 1 # in seconds
max_pwm_val = 250
pwm_step_size = 10
current_pwm = max_pwm_val
current_index = 0
avg_speedM1 = 1 # initialize to non zero value to avoid zero test in while loop
avg_speedM2 = 1
avg_speedM3 = 1

# Store the table values
table_pwm = []
table_speedsM1 = []
table_speedsM2 = []
table_speedsM3 = []

while current_pwm >= 0 and not (avg_speedM1 + avg_speedM2 + avg_speedM3) == 0:
	# Set the power
	setPower(current_pwm, current_pwm, current_pwm)
	print(current_pwm)

	# Wait for speed to level
	time.sleep(speed_settle_wait_time)

	# Read the average speed
	speedsM1 = []
	speedsM2 = []
	speedsM3 = []
	for i in range(0, 50):
		speed_values = getSpeed()
		speedsM1.append(speed_values[0])
		speedsM2.append(speed_values[1])
		speedsM3.append(speed_values[2])
	avg_speedM1 = np.mean(speedsM1)
	avg_speedM2 = np.mean(speedsM2)
	avg_speedM3 = np.mean(speedsM3)

	# Store values
	storeLookupValue(1, current_pwm, avg_speedM1, current_index) # Store on PSoC
	storeLookupValue(2, current_pwm, avg_speedM2, current_index)
	storeLookupValue(3, current_pwm, avg_speedM3, current_index)

	table_pwm.append(current_pwm) # Store locally
	table_speedsM1.append(avg_speedM1)
	table_speedsM2.append(avg_speedM2)
	table_speedsM3.append(avg_speedM3)

	# Increase counters
	index += 1
	current_pwm -= pwm_step_size

disengage()

table_data = np.array([table_pwm, table_speedsM1, table_speedsM2, table_speedsM3)
np.save('bang_lookup_table.npy', table_data)

# Plot table values
plt.plot(table_pwm, table_speedsM1) #blue
plt.plot(table_pwm, table_speedsM2) #green
plt.plot(table_pwm, table_speedsM3) #red
plt.legend(['motor1', 'motor2', 'motor3'], loc='lower right')

plt.ylabel('Wheel speeds')
plt.xlabel('PWM')
plt.show()
