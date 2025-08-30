function plotResults(refMap, robotTrajectories, globalMap, objectLocations, detectedObjects, V_total)
% PLOTRESULTS Show final outputs: trajectories, merged map, potential energy.

    % Final trajectories overlayed on ground truth map
    figure('Name','Final Robot Trajectories');
    hold on; grid on;
    show(refMap);
    for r = 1:numel(robotTrajectories)
        traj = robotTrajectories{r};
        plot(traj(:,1), traj(:,2), 'LineWidth', 1.5, 'DisplayName', sprintf('Robot %d', r));
        plot(traj(1,1), traj(1,2), 'go', 'MarkerSize',8);
        plot(traj(end,1), traj(end,2), 'rx', 'MarkerSize',8);
    end
    legend(); title('Final Robot Trajectories');
    hold off;

    % Merged map with detected objects
    figure('Name','Merged Map');
    show(globalMap); hold on;
    for o = 1:size(objectLocations,1)
        if detectedObjects(o)
            plot(objectLocations(o,1), objectLocations(o,2), 'rx', 'MarkerSize',10, 'LineWidth',2);
        else
            plot(objectLocations(o,1), objectLocations(o,2), 'kx');
        end
    end
    title('Merged Map with Detected Objects');
    hold off;

    % Potential energy over time
    if ~isempty(V_total)
        figure('Name','Potential Energy');
        plot(1:length(V_total), V_total, '-o');
        xlabel('Time step'); ylabel('Total Potential Energy');
        title('Total Potential Energy Over Time'); grid on;
    end
end
