function newPose = levyWalkWithPotentialField(pose, config, dynamicMap, robotPoses, robotIndex)
% LEVYWALKWITHPOTENTIALFIELD Compute next pose using Lévy walk + potential fields.
%
% Inputs:
%   pose        - [x y theta] of the current robot
%   config      - config struct
%   dynamicMap  - binaryOccupancyMap used to check obstacles
%   robotPoses  - Nx3 array of all robot poses
%   robotIndex  - index of the robot being updated
%
% Output:
%   newPose     - [x y theta] updated pose suggestion

    % Draw Levy step length
    alpha = config.levyAlpha;
    stepLength = config.stepSize * (rand()^(1/(-alpha)));
    direction = 2 * pi * rand();

    % Attraction (towards Levy target)
    attractiveForce = [cos(direction), sin(direction)] * stepLength;

    % Repulsive forces from other robots
    repulsiveForce = [0, 0];
    maxRepulsiveForce = 7;
    influenceRange = config.repulsionRange;
    for r = 1:size(robotPoses, 1)
        if r ~= robotIndex
            diff = robotPoses(robotIndex,1:2) - robotPoses(r,1:2);
            dist = norm(diff);
            if dist > 0 && dist < influenceRange
                strength = maxRepulsiveForce * (1 - dist / influenceRange);
                repulsiveForce = repulsiveForce + strength * (diff / (dist + eps));
            end
        end
    end

    % Small random drift
    driftForce = 0.09 * [randn(), randn()];

    % Combine and limit
    attractiveWeight = 1.0;
    repulsiveWeight = 4.0;
    totalForce = attractiveWeight * attractiveForce + repulsiveWeight * repulsiveForce + driftForce;

    if norm(totalForce) > config.stepSize
        totalForce = totalForce / norm(totalForce) * config.stepSize;
    end

    % Proposed new position
    newPose = pose;
    newPose(1) = pose(1) + totalForce(1);
    newPose(2) = pose(2) + totalForce(2);

    % Keep inside bounds
    newPose(1) = max(min(newPose(1), config.mapdimx - 1), 1);
    newPose(2) = max(min(newPose(2), config.mapdimy - 1), 1);

    % If proposed cell is occupied in dynamicMap, try rotating slightly until free
    attempts = 0;
    while getOccupancy(dynamicMap, [newPose(1), newPose(2)]) == 1 && attempts < 16
        direction = direction + pi/8; % rotate 22.5 degrees
        newPose(1) = pose(1) + stepLength * cos(direction);
        newPose(2) = pose(2) + stepLength * sin(direction);
        newPose(1) = max(min(newPose(1), config.mapdimx - 1), 1);
        newPose(2) = max(min(newPose(2), config.mapdimy - 1), 1);
        attempts = attempts + 1;
    end

    % Update orientation
    newPose(3) = atan2(newPose(2) - pose(2), newPose(1) - pose(1));
end
