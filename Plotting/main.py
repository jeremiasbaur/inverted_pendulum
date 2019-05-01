import serial, time
import pandas as pd

df = pd.DataFrame(columns = ['Desired angle', 'Error angle', 'Angle', 'Position', 'Average angle velocity', 'Average rail velocity', 'Motor output', 'Time'])

df = df.append(pd.Series([0,0,0,0,0,0,0,0], index = ['Desired angle', 'Error angle', 'Angle', 'Position', 'Average angle velocity', 'Average rail velocity', 'Motor output', 'Time']), ignore_index=True)

print(df)

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
