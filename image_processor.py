import cv2
import sys
import time
import serial
import numpy as np

threshold = 60

serial_port = "/dev/ttyS0" # If you are using built_in serial interface for LEONARDO connection
#serial_port = "/dev/ttyAMA0" #If you are using USB serial interface for LEONARDO connection

br = 57600 # Arduino Leonardo's baudrate. 57600 is recommended.

# Get the camera ready
def initialize_camera():
    global video
    # Select the camera
    video = cv2.VideoCapture(0)


# Calculate unstable angles into stables.
def calcang(w_min, h_min, ang):
    if ang < -45:
        ang = 90 + ang
    if w_min < h_min and ang > 0:
        ang = (90 - ang) * -1
    if w_min > h_min and ang < 0:
        ang = 90 + ang

    return ang

# Sort rectangles by ratio and return thee rectangle index of maximum ratio.
def findmaxratio(rectangle):

    retindex = 0
    maxratio = 0
    index = -1

    for rect in rectangle:

        currentratio = 0
        index += 1

        (rec_x, rec_y), (rec_w, rec_h), rec_ang = rect
        if rec_w > rec_h:
            currentratio = rec_w / rec_h
        elif rec_h > rec_w:
            currentratio = rec_h / rec_w

        if currentratio > maxratio:
            maxratio = currentratio
            retindex = index

    if index <= 0:
        index = 0
    print("possible lines count: " + str(index))
    return retindex


def establish_serial_connection():
    global si
    try:
        si = serial.Serial(serial_port, baudrate=br, timeout=1)

        si.write(b"<rpi_ready>")
        while not si.read().decode("utf-8") == "<ard_booted>":
            print("Waiting for heartbeat from Arduino...")
            time.sleep(0.1)
    except serial.SerialException:
        print("Serial port", serial_port, "is not accessible. Program exiting...")
        sys.exit(-1)



# Main loop to process frames and send it via serial interface.
def main_loop():
    global si
    while True:

        # Capture frame
        check, frame = video.read()
        setpoint = 320 # center in width

        blackline = cv2.inRange(frame, (0, 0, 0), (threshold, threshold, threshold)) # Convert image to white/black

        # Denoise image
        kernel = np.ones((3, 3), np.uint8)
        blackline = cv2.erode(blackline, kernel, iterations=5)
        blackline = cv2.dilate(blackline, kernel, iterations=9)

        # Select contours via builtin opencv function
        contours_blk, hierarchy_blk = cv2.findContours(blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Convert contour to rectangles to calculate ratio. This logic is based on lines are long and thin
        alist = []
        index = -1
        for contour in contours_blk:
            index += 1
            blackbox = cv2.minAreaRect(contour)

            alist.append(blackbox)

        maxratin = findmaxratio(alist)

        try:
            # Select the rectangle with max ratio
            (boxx, boxy), (boxw, boxh), boxang = alist[maxratin]

            # Calculate needed informations
            error = int(boxx - setpoint + 320)
            boxang = int(boxang)
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)

            error = str(error)
            angle = str(calcang(boxw, boxh, boxang) + 90)

            # Draw OSD
            cv2.line(frame, (320, 0), (320, 480), (76, 0, 153), 2)  # Center Guideline
            cv2.line(frame, (0, 240), (640, 240), (76, 0, 153), 2)  # Center Guideline

            cv2.drawContours(frame, [box], 0, (0, 0, 150), 3)  # Contour the Trackline

            cv2.line(frame, (int(boxx), int(boxy)), (320, int(boxy)), (0, 255, 0), 2)
            cv2.putText(frame, "distance: " + error, (int((320 + boxx) / 2), int(boxy) - 30), cv2.FONT_HERSHEY_SIMPLEX,
                        .7, (100, 255, 0), 2)  # Distance from Trackline
            cv2.putText(frame, "angle: " + angle, (int((320 + boxx) / 2), int(boxy) - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 100), 2)  # Angle from Trackline

            data_to_send = "<" + error.zfill(3) + ":" + angle.zfill(3) + ">"
            si.write(bytes(data_to_send, encoding="utf-8"))

            respond = si.read().decode("utf-8")
            while not respond == "<read>":
                respond = si.read().decode("utf-8")
                print("Arduino did not receive ACK package. Waiting...")


        except:
            cv2.putText(frame, "NO LINE DETECTED", (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)  # Angle from Trackline
            print("No line detected.")

            data_to_send = "<000:999>"
            si.write(bytes(data_to_send, encoding="utf-8"))

            respond = si.read().decode("utf-8")
            while not respond == "<read>":
                respond = si.read().decode("utf-8")
                print("Arduino did not receive ACK package. Waiting...")

        # Display on screen
        cv2.imshow("Output", frame)

initialize_camera()
establish_serial_connection()
main_loop()