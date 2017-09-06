#!/usr/bin/python
import serial, struct

ser = serial.Serial('/dev/ttyUSB0', 9600) 
# print(ser.name)
ser.flushInput()
ser.flushOutput()
while(1):
	bytesToRead = ser.inWaiting()
	byte = ser.read(bytesToRead)
	with open ("decoder.txt",'a') as writer:
		data = ser.read(46)
		crcComp = reduce(lambda x, y: x + ord(y), data[0:45], 0)%256
		sync, nr, b1, b2, g1, g2, g3, s1, s2, s3, a1, a2, a3, crc = struct.unpack('<2HfB3i6fB', data)
		out = hex(sync) + "," + str(nr) + "," + str(b1) + "," + str(b2) + "," + str(g1) + "," + str(g2) + "," + str(g3) + "," + str(s1) + "," + str(s2) + "," + str(s3) + "," + str(a1) + "," + str(a2) + "," + str(a3) + ',' + str(crcComp-crc) + "\n"
		writer.write(out)
		print out
