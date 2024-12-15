# Maze Solver and Robot Calibration Project

This project involves solving a maze and calibrating a camera for capturing and processing maze images. It also includes inverse kinematics (IK) and forward kinematics (FK) solvers for a robotic arm.

## Project Structure

```
.DS_Store
calibrate.py
camera_matrix.npy
captureMaze.py
dist_coeffs.npy
IKSolver.m
Maze/
MazeSolver.py
mazeSolverV2.py
mycobot_pro_600/
    .DS_Store
    base.stl
    link1.stl
    link2.stl
    link3.stl
    link4.stl
    link5.stl
    link6.stl
mycobot_pro_600.urdf
Readme.md
Solved/
Real_World_Waypoints.txt
```

## Files and Directories

- **calibrate.py**: Script for calibrating the camera using a checkerboard pattern.
- **camera_matrix.npy**: Saved camera matrix from calibration.
- **captureMaze.py**: Script to capture and undistort maze images using the calibrated camera.
- **dist_coeffs.npy**: Saved distortion coefficients from calibration.
- **IKSolver.m**: MATLAB script for inverse kinematics and forward kinematics solvers.
- **Maze/**: Directory containing maze images.
- **MazeSolver.py**: Script to solve the maze and generate waypoints.
- **mazeSolverV2.py**: Another version of the maze solver script.
- **mycobot_pro_600/**: Directory containing 3D models and URDF file for the robotic arm.
- **Solved/**: Directory containing the solved maze image and real-world waypoints.
- **Readme.md**: This readme file.

## Usage

### Camera Calibration

1. Run `calibrate.py` to calibrate the camera using a checkerboard pattern.
2. Follow the on-screen instructions to capture images.
3. The calibration results will be saved in `camera_matrix.npy` and `dist_coeffs.npy`.

### Capture and Undistort Maze Image

1. Run `captureMaze.py` to capture and undistort an image of the maze.
2. The undistorted image will be saved as `Captured_Maze_Undistorted.png`.

### Solve the Maze

1. Place the maze image in the `Maze/` directory.
2. Run `MazeSolver.py` to solve the maze and generate waypoints.
3. The solved maze image and waypoints will be saved in the `Solved/` directory.

### Inverse Kinematics

1. Run `IKSolver.m` in MATLAB to compute the joint angles for the waypoints.
2. The joint angles will be displayed in the MATLAB console.

## Dependencies

- Python 3.x
- OpenCV
- NumPy
- MATLAB (for `IKSolver.m`)

## License

This project is licensed under the MIT License.
