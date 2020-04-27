import numpy as np
import cv2
import serial
import time
import csv
import keyboard

target_fps = 30.0
frame_period = 1.0/target_fps

ser=serial.Serial("/dev/ttyUSB0",115200)
ser.baudrate = 115200

cap = cv2.VideoCapture(0)

if (cap.isOpened() == False):
	print("Error opening video stram or file")

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

file = open("test_csv.csv",'w',newline='')
writer = csv.writer(file)

row_headers = ['T','D','L','Ly','G','Gy','Gz','A','Ay','Az','M','My','Mz','P','R','H']
blank_row = ['','','','','','','','','','','','','','','','']
writer.writerow(row_headers)

out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'),30, (frame_width, frame_height))
i = 0
csv_line = blank_row
first_off_session = True
second_off_session = False
take_video = False
last_time = time.perf_counter()

print("start")
while(cap.isOpened()):
	if (ser.inWaiting() > 0):
		read_ser = ser.read(ser.inWaiting())
		read_ser = read_ser.decode('ascii')

		if (read_ser[:3] != "OFF" and read_ser != ""):

			lines = read_ser.split('\n')
			for line in lines:
				line = line[:len(line)-1]
				input_str = line.split(':')
				if (len(input_str) > 1):
					data_str = input_str[1].split(',')
					if input_str[1][:5] == "GPGGA":

						if (len(data_str) > 6):
							data_str = [data_str[2], data_str[4]]
							if (input_str[0] in row_headers):
								csv_line[2:4] = data_str
					else:					
						if (input_str[0] in row_headers):
								ind = row_headers.index(input_str[0])
								csv_line[ind:(ind + len(data_str))] = data_str
								take_video = True

					if (input_str[0] == 'H'):
						writer.writerow(csv_line)
						csv_line = blank_row
						first_off_session = False
						
		
		if (read_ser[:3] == "OFF"):
			if (first_off_session == False):
				print("breaking")
				break

	if (time.perf_counter() - frame_period > last_time):
		last_time = time.perf_counter()
		ret, frame = cap.read()
		if (ret == True and take_video == True):
			out.write(frame)
		if keyboard.is_pressed('q'):
			print("quitting")
			break

# When everything done, release the capture

cap.release()
out.release()
cv2.destroyAllWindows()

