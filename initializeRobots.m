function [robotPoses, controllers, sensors, robotKinematics] = initializeRobots(numRobots, refMap)
% INITIALIZEROBOTS Sets initial poses, controllers, sensors, and kinematics.
%
% Inputs:
%   numRobots - number of robots
%   refMap    - ground truth map for collision checks
%
% Outputs:
%   robotPoses      - Nx3 [x y theta]
%   controllers     - cell array of controllerPurePursuit objects
%   sensors         - cell array of rangeSensor objects
%   robotKinematics - differentialDriveKinematics object

    % Predefined initial positions (ensure these are in free space)
    initialPositions = [
        5, 5;
        20, 4;
        9, 23;
        22, 22
    ];

    if numRobots > size(initialPositions, 1)
        error('Not enough predefined initial positions for %d robots.', numRobots);
    end

    robotPoses = zeros(numRobots, 3);
    for r = 1:numRobots
        x = initialPositions(r,1);
        y = initialPositions(r,2);
        % Validate free space
        if getOccupancy(refMap, [x, y]) == 0
            robotPoses(r, :) = [x, y, 2*pi*rand()];
        else
            error('Initial position [%f, %f] is inside an obstacle.', x, y);
        end
    end

    % Controllers (pure pursuit) and sensors
    controllers = arrayfun(@(x) controllerPurePursuit('DesiredLinearVelocity', 0.3, ...
        'MaxAngularVelocity', 0.8), 1:numRobots, 'UniformOutput', false);

    sensors = arrayfun(@(x) rangeSensor('Range', [0, 7]), 1:numRobots, 'UniformOutput', false);

    % Kinematics model
    robotKinematics = differentialDriveKinematics("VehicleInputs", "VehicleSpeedHeadingRate");
end
