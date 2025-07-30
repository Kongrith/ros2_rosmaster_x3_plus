import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture("USBserial")   // 0

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # If frame is not read correctly, break the loop
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Display the resulting frame
    cv2.imshow('Live Feed', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and destroy all windows
cap.release()
cv2.destroyAllWindows()