#import serial, time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rc
from pylab import *
import sys

rc('text', usetex=True)

df = pd.DataFrame(columns = ['Desired angle', 'Error angle', 'Angle', 'Position', 'Average angle velocity', 'Average rail velocity', 'Motor output', 'Time', 'calc_angle_velocity'])

counter = 0
with open(r'./Logs/'+sys.argv[1], 'r') as file:
	file.readline()
	file.readline()
	last = [None,None]
	for line in file.readlines():
		values = line.split('\t')
		for i in range(len(values)):
			values[i] = float(values[i])
		if last[0] != None:
			values.append((values[2] - last[0]) / (values[7] - last[1]))
			df = df.append(pd.Series(values , index = ['Desired angle', 'Error angle', 'Angle', 'Position', 'Average angle velocity', 'Average rail velocity', 'Motor output', 'Time', 'calc_angle_velocity']), ignore_index=True)
		last = [values[2], values[7]]
		if counter > int(sys.argv[2]): break
		counter+=1

df['Angle'] = df['Angle'].apply(lambda x : (360/1200)*x)
df['Average angle velocity'] = df['Average angle velocity'].apply(lambda x : (2*3.141/1200)*x*1000)
df['Position'] = df['Position'].apply(lambda x : (0.05/1589)*x)
df['Average rail velocity'] = df['Average rail velocity'].apply(lambda x : (0.05/1589)*x*1000)

#df = df.append(pd.Series([0,0,0,0,0,0,0,0], index = ['Desired angle', 'Error angle', 'Angle', 'Position', 'Average angle velocity', 'Average rail velocity', 'Motor output', 'Time']), ignore_index=True)

#print(df)

with_line = int(sys.argv[3]) == 1


fig = plt.figure(figsize=(16,8))

descriptions = [['Angle', r"$t$ in $ms$", r"$\theta$ in Grad",'(a)'],
				['Position', r"$t$ in $ms$", r"$x$ in Meter",'(c)'],
				['Average angle velocity', r"$t$ in $ms$", r"$\dot\theta$ in $rad/s$",'(b)'],
				['Average rail velocity', r"$t$ in $ms$", r"$\dot x$ in $m/s$",'(d)']]

for i in range(4):
	sub = plt.subplot(2, 2, i+1)
	sub = df.reset_index().plot(ax = sub, x = 'Time', y = descriptions[i][0], kind='scatter', s=1)
	if with_line:
		sub = df.reset_index().plot(ax = sub, x = 'Time', y = descriptions[i][0], kind='line', linewidth=0.3)
		sub.get_legend().remove()
	xlabel(descriptions[i][1])
	ylabel(descriptions[i][2])
	title(descriptions[i][3])
	plt.tight_layout()
	sub.grid(True)

if int(sys.argv[4]) == 1:
	plt.savefig(r'./Logs/'+sys.argv[5])
plt.show()
"""
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM28'
ser.open()

time.sleep(3)

#values = bytearray([4, 9, 62, 144, 56, 30, 147, 3, 210, 89, 111, 78, 184, 151, 17, 129])
#ser.write(values)
 
data = ser.read(size = ser.in_waiting)
print(data)
print(type(data))

#for byte in data:
#	print(chr(byte),end='')

ser.close()
"""