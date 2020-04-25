import cv2
import numpy as np
import time

cap = cv2.VideoCapture('output.avi')
fps = 30.0
fps_period = 1.0/fps
while(True):
	ret, frame = cap.read()

	if ret == True:

		cv2.imshow('frame',frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	else:
		break
	
	time.sleep(fps_period)

cap.release()
cv2.destroyAllWindows()
