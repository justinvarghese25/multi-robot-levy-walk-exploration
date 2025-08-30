function [robotTrajectories, globalMap, V_total] = runSimulation(config, refMap, dynamicMap, ...
    robotMaps, globalMap, objectLocations, detectedObjects, robotPoses, controllers, sensors, robotKinematics)
% RUNSIMULATION Main simulation loop for multi-robot exploration.
%
% Outputs:
%   robotTrajectories - cell array of trajectories for each robot
%   globalMap         - final merged map
%   V_total           - potential energy values over time

    V_total = [];
    robotTrajectories = cell(config.numRobots, 1);
    for r = 1:config.numRobots
        robotTrajectories{r} = robotPoses(r, 1:2);
    end

    tic;
    while toc < config.simulationTime
        elapsedTime = toc;
        fprintf('Time Elapsed: %.2f seconds\n', elapsedTime);

        % Update each robot sequentially (simulates parallel updates)
        for r = 1:config.numRobots
            % Obtain simulated range readings from the true map
            [ranges, angles] = sensors{r}(robotPoses(r, :), refMap);
            scan = lidarScan(ranges, angles);
            validScan = removeInvalidData(scan, 'RangeLimits', sensors{r}.Range);

            % Insert rays into dynamic and local maps
            insertRay(dynamicMap, robotPoses(r, :), validScan, sensors{r}.Range(2));
            insertRay(robotMaps{r}, robotPoses(r, :), validScan, sensors{r}.Range(2));

            % Object detection
            for objIdx = 1:size(objectLocations, 1)
                if ~detectedObjects(objIdx)
                    distanceToObject = norm(robotPoses(r, 1:2) - objectLocations(objIdx, :));
                    if distanceToObject < config.detectionRadius
                        detectedObjects(objIdx) = true;
                        setOccupancy(robotMaps{r}, objectLocations(objIdx, :), 1);
                    end
                end
            end

            % Compute next waypoint using Levy + potential field
            nextPose = levyWalkWithPotentialField(robotPoses(r, :), config, dynamicMap, robotPoses, r);

            % Obstacle avoidance check: if no front readings, treat as clear
            while true
                [ranges_d, angles_d] = sensors{r}(robotPoses(r, :), dynamicMap);
                frontIndices = find(angles_d >= -pi/6 & angles_d <= pi/6);
                if isempty(frontIndices)
                    frontRange = inf;
                else
                    frontRange = min(ranges_d(frontIndices));
                end

                if frontRange < config.wallDistance
                    % Turn in place slightly and recompute
                    robotPoses(r,3) = robotPoses(r,3) + pi/4;
                    nextPose = levyWalkWithPotentialField(robotPoses(r, :), config, dynamicMap, robotPoses, r);
                    controllers{r}.Waypoints = nextPose(1:2);
                    continue;
                else
                    controllers{r}.Waypoints = nextPose(1:2);
                end

                % Compute control and update pose
                [v, w] = controllers{r}(robotPoses(r, :));
                vel = derivative(robotKinematics, robotPoses(r, :), [v, w]);
                robotPoses(r, :) = robotPoses(r, :) + vel' * config.sampleTime;

                % Keep inside bounds
                robotPoses(r, 1) = max(min(robotPoses(r, 1), config.mapdimx - 1), 1);
                robotPoses(r, 2) = max(min(robotPoses(r, 2), config.mapdimy - 1), 1);

                % Append trajectory point
                robotTrajectories{r} = [robotTrajectories{r}; robotPoses(r, 1:2)];

                % Exit movement loop if close to the waypoint
                if norm(robotPoses(r, 1:2) - nextPose(1:2)) < 0.1
                    break;
                end
            end
        end

        % Merge local maps into global map
        globalMap = mergeRobotMaps(robotMaps, objectLocations, detectedObjects);

        % Compute total potential energy for diagnostics
        V_step = 0;
        for ri = 1:config.numRobots
            % For attractive part use distance to its last computed nextPose approx.
            % (use robot's current waypoint as a proxy)
            d_a = 0.5 * norm(controllers{ri}.Waypoints - robotPoses(ri,1:2))^2;
            V_r = 0;
            for j = 1:config.numRobots
                if j ~= ri
                    d_r = norm(robotPoses(ri,1:2) - robotPoses(j,1:2));
                    if d_r > 0 && d_r < config.repulsionRange
                        V_r = V_r + (config.safeDistance / d_r)^2;
                    end
                end
            end
            V_step = V_step + (d_a + V_r);
        end
        V_total = [V_total, V_step];

        % Visualization (quick)
        clf;
        show(globalMap);
        title(sprintf('Dynamic Map - Time: %.2f s', elapsedTime));
        hold on;

        % Plot objects and robot poses
        for o = 1:size(objectLocations,1)
            if detectedObjects(o)
                plot(objectLocations(o,1), objectLocations(o,2), 'rx', 'MarkerSize',10,'LineWidth',2);
            else
                plot(objectLocations(o,1), objectLocations(o,2), 'kx', 'MarkerSize',8);
            end
        end
        for rr = 1:config.numRobots
            plot(robotPoses(rr,1), robotPoses(rr,2), 'bo', 'MarkerSize',8, 'LineWidth',1.5);
            dirX = robotPoses(rr,1) + 1.5*cos(robotPoses(rr,3));
            dirY = robotPoses(rr,2) + 1.5*sin(robotPoses(rr,3));
            plot([robotPoses(rr,1), dirX], [robotPoses(rr,2), dirY], 'r-');
        end
        drawnow;

        % small pause to avoid hogging CPU (adjustable)
        pause(0.001);
    end
end
