import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break

    # Our operations on the frame come here
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Display the resulting frame

    blue_low = np.array([90,50, 50])
    blue_high = np.array([130, 255, 255])

    mask = cv2.inRange(hsv, blue_low, blue_high)

    kern = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
    make = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kern)


    cntr, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    count = 0

    for i,c in enumerate(cntr):
        if cv2.contourArea(c) > 2500:
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 255, 0), 2)
            print(f"Object {i} at ({(x+w)/2}, {(y +h)/2 })")
    
    cv2.imshow('frame',frame)
    cv2.imshow("mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()