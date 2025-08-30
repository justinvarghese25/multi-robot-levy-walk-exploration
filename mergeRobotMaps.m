function mergedMap = mergeRobotMaps(robotMaps, objectLocations, detectedObjects)
% MERGEROBOTMAPS Merge local occupancy matrices into a global binaryOccupancyMap.
%
% Inputs:
%   robotMaps      - cell array of binaryOccupancyMap objects
%   objectLocations- Nx2 positions to mark if detected
%   detectedObjects- Nx1 logical flags
%
% Output:
%   mergedMap      - binaryOccupancyMap with merged occupancy

    % Convert each robot map to occupancy matrix (same resolution assumed)
    mapGrids = cellfun(@occupancyMatrix, robotMaps, 'UniformOutput', false);
    mergedGrid = max(cat(3, mapGrids{:}), [], 3); % element-wise max across stacks

    % Build binaryOccupancyMap from merged grid. occupancyMatrix returns values
    % in same sampling resolution as map, so pass resolution from robotMaps{1}
    res = robotMaps{1}.Resolution;
    mergedMap = binaryOccupancyMap(mergedGrid, res);

    % Mark detected objects as occupied
    for i = 1:size(objectLocations, 1)
        if detectedObjects(i)
            % setOccupancy expects [x,y] in map coordinates
            setOccupancy(mergedMap, objectLocations(i, :), 1);
        end
    end
end
