import cv2
import numpy as np
import time

# ESP32-CAM Stream URL
STREAM_URL = 'http://192.168.1.216:81/stream'

def main():
    print("Agentic Pet OS - Vision Node Initializing...")
    cap = cv2.VideoCapture(STREAM_URL)

    if not cap.isOpened():
        print("Failed to open the camera stream. Is the ESP32-CAM running and on the same network?")
        return

    print("Vision Stream Connected! Press 'q' to quit.")

    # Background subtractor for motion detection
    fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Retrying in 1 second...")
            time.sleep(1)
            # Try to reconnect
            cap.release()
            cap = cv2.VideoCapture(STREAM_URL)
            continue

        # Get frame dimensions
        height, width, _ = frame.shape
        center_x = width // 2

        # Apply background subtraction
        fgmask = fgbg.apply(frame)

        # Clean up the mask with morphology
        kernel = np.ones((5,5), np.uint8)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)

        # Find contours of moving objects
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = None
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum size to be considered 'motion'
                if area > max_area:
                    max_area = area
                    largest_contour = contour

        command = "STOP"

        if largest_contour is not None:
            # Get the bounding box of the moving object
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Calculate object center
            obj_cx = x + (w // 2)

            # Determine turning command based on object position
            deadzone = 50 # Deadzone in the center where it won't turn
            if obj_cx < (center_x - deadzone):
                command = "TURN LEFT"
                cv2.putText(frame, "<-- TRACK LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif obj_cx > (center_x + deadzone):
                command = "TURN RIGHT"
                cv2.putText(frame, "TRACK RIGHT -->", (width - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                command = "FORWARD (CHASE)"
                cv2.putText(frame, "^^^ CHASE ^^^", (center_x - 100, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        else:
            cv2.putText(frame, "IDLE - SEARCHING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        # Show the frame
        cv2.imshow('Pet OS Vision', frame)
        # cv2.imshow('Motion Mask', fgmask) # Uncomment to see the raw motion mask

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
