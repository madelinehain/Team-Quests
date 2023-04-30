from imutils.video import VideoStream
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import time
import cv2

# Detup CLI argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="barcodes.csv",
    help="path to output CSV file containing barcodes")
args = vars(ap.parse_args())

# Initialize the Pi cam video stream
#vs = VideoStream(src=0).start()  #Uncomment this if you are using Webcam
vs = VideoStream(usePiCamera=True).start() # For Pi Camera
time.sleep(2.0)
csv = open(args["output"], "w")
found = set()

print("Initialized")

# Big ol' reccuring loop
while True:
    # Detect and decode QR code
    frame = vs.read()
    frame = imutils.resize(frame, width=400)
    barcodes = pyzbar.decode(frame)
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        text = "{} ({})".format(barcodeData, barcodeType)
        print (text)
        cv2.putText(frame, text, (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # Show the barcode and barcode information
        cv2.imshow("Barcode Reader", frame)

    # if the barcode text is currently not in our CSV file, write
    # the timestamp + barcode to disk and update the set
        if barcodeData not in found:
            csv.write("{},{}\n".format(datetime.datetime.now(), barcodeData))
        csv.flush()
        found.add(barcodeData)
        cv2.imshow("Barcode Reader", frame)

    # if the `s` key is pressed, break from the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("s"):
        break

# Clean-up procedures
print("[INFO] cleaning up...")
csv.close()
cv2.destroyAllWindows()
vs.stop()