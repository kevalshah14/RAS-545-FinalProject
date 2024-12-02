import cv2
import numpy as np

def load_calibration(calibration_file_camera_matrix='camera_matrix.npy',
                    calibration_file_dist_coeffs='dist_coeffs.npy'):
    """
    Loads the camera calibration data.

    Parameters:
    - calibration_file_camera_matrix: Path to the saved camera matrix.
    - calibration_file_dist_coeffs: Path to the saved distortion coefficients.

    Returns:
    - camera_matrix: Intrinsic camera matrix.
    - dist_coeffs: Distortion coefficients.
    """
    try:
        camera_matrix = np.load(calibration_file_camera_matrix)
        dist_coeffs = np.load(calibration_file_dist_coeffs)
        print("Calibration data loaded successfully.")
        print("Camera matrix:\n", camera_matrix)
        print("Distortion coefficients:\n", dist_coeffs)
        return camera_matrix, dist_coeffs
    except FileNotFoundError as e:
        print(f"Calibration file not found: {e}")
        return None, None

def capture_and_undistort_image(camera_index=1, calibration_data=None, save_path='Captured_Maze.png'):
    """
    Captures an image from the external camera, undistorts it, and saves the result.

    Parameters:
    - camera_index: Index of the camera (default is 0).
    - calibration_data: Tuple containing (camera_matrix, dist_coeffs).
    - save_path: Path to save the undistorted image.

    Returns:
    - undistorted_img: The undistorted image.
    """
    if calibration_data is None:
        print("Calibration data is required for undistortion.")
        return None

    camera_matrix, dist_coeffs = calibration_data

    # Initialize camera
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera with index {camera_index}")
        return None

    print("Capturing image. Press 'c' to capture or 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        cv2.imshow('Camera Feed - Press "c" to Capture', frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('c'):
            # Capture the frame
            print("Image captured.")
            break
        elif key & 0xFF == ord('q'):
            print("Capture canceled.")
            cap.release()
            cv2.destroyAllWindows()
            return None

    cap.release()
    cv2.destroyAllWindows()

    # Undistort the captured image
    undistorted_img = cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix)

    # Save the undistorted image
    cv2.imwrite(save_path, undistorted_img)
    print(f"Undistorted image saved to {save_path}")

    # Optional: Display the undistorted image
    # cv2.imshow('Undistorted Image', undistorted_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return undistorted_img

if __name__ == "__main__":
    # Load calibration data
    calibration_data = load_calibration()

    if calibration_data[0] is not None and calibration_data[1] is not None:
        # Capture and undistort image
        captured_image = capture_and_undistort_image(
            camera_index=0,  # Adjust if you have multiple cameras
            calibration_data=calibration_data,
            save_path='Captured_Maze_Undistorted.png'
        )
    else:
        print("Cannot proceed without calibration data.")
