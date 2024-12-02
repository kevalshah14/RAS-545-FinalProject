import cv2
import numpy as np

# Set up criteria for termination of corner sub-pixel accuracy search
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points for a checkerboard
checkerboard_size = (8, 6)  # Internal corners per chessboard row and column
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Initialize webcam
cap = cv2.VideoCapture(0)  # Change to 0 if using the default webcam

if not cap.isOpened():
    print("Error: Unable to access the webcam.")
    exit()

print("Press 's' to capture an image, 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if found:
        # Draw the corners on the frame
        cv2.drawChessboardCorners(frame, checkerboard_size, corners, found)
        cv2.putText(frame, "Checkerboard detected. Press 's' to save.", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "No checkerboard detected. Adjust the view.", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Display the live video feed
    cv2.imshow('Live Video - Press "s" to capture, "q" to quit', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and found:  # Save image if 's' is pressed and checkerboard is detected
        print("Checkerboard captured!")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        print(f"Captured {len(objpoints)}/10 images.")
        if len(objpoints) >= 10:
            print("Captured all required images. Exiting...")
            break
    elif key == ord('q'):  # Quit the capture loop
        print("Exiting without completing calibration.")
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) >= 10:
    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Save the calibration results
    np.save("camera_matrix.npy", camera_matrix)
    np.save("dist_coeffs.npy", dist_coeffs)

    print("Calibration completed successfully.")
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)
else:
    print("Calibration incomplete. Not enough images captured.")
