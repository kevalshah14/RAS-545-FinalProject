function output = my_kinematics(mode, input)
    if strcmp(mode, 'ik')
        output = ik_solver(input);
    elseif strcmp(mode, 'fk')
        output = fk_solver(input);
    else
        error('Invalid mode. Use "ik" or "fk".');
    end
end

function jointAngles = ik_solver(waypoints)
    % Fixed z-coordinate for all waypoints
    fixedZ = 10;
    eulerAngles = [178, -0.0, 0];
    eulerAnglesRad = deg2rad(eulerAngles);

    % Import the robot model
    robot = importrobot("mycobot_pro_600/mycobot_pro_600.urdf");
    gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'pose'});
    gik.SolverParameters.MaxIterations = 500;
    initialGuess = homeConfiguration(robot);

    numWaypoints = size(waypoints, 1);
    numJoints = numel(homeConfiguration(robot));
    jointAngles = zeros(numWaypoints, numJoints);

    for i = 1:numWaypoints
        position = [waypoints(i, :), fixedZ]; % Add fixed z-coordinate
        tform = eul2tform(eulerAnglesRad, 'XYZ');
        tform(1:3, 4) = position;

        poseConstraint = constraintPoseTarget('link6');
        poseConstraint.TargetTransform = tform;

        [configSoln, ~] = gik(initialGuess, poseConstraint);
        jointAngles(i, :) = rad2deg([configSoln.JointPosition]);
        initialGuess = configSoln;
    end
end

function endEffectorPose = fk_solver(jointAngles)
    robot = importrobot("mycobot_pro_600/mycobot_pro_600.urdf");

    config = homeConfiguration(robot);
    for i = 1:numel(jointAngles)
        config(i).JointPosition = deg2rad(jointAngles(i));
    end

    endEffectorPose = getTransform(robot, config, 'link6');
end

waypoints = [
    np.float32(0.125), np.float32(0.125);
    np.float32(2.875), np.float32(0.125);
    np.float32(2.875), np.float32(4.875);
    np.float32(7.625), np.float32(4.875);
    np.float32(7.625), np.float32(7.125);
    np.float32(2.375), np.float32(7.125);
    np.float32(2.375), np.float32(9.625);
    np.float32(5.125), np.float32(9.625);
    np.float32(5.125), np.float32(9.875);
    np.float32(7.125), np.float32(9.875)
];

% Call the IK solver to get joint angles
jointAngles = my_kinematics('ik', waypoints);
disp('Joint Angles for Waypoints:');
disp(jointAngles);

