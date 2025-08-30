function [refMap, dynamicMap, config, robotMaps, globalMap, objectLocations, detectedObjects] = initializeEnvironment()
% INITIALIZEENVIRONMENT Loads example map and sets simulation parameters.
%
% Outputs:
%   refMap         - Ground truth binaryOccupancyMap
%   dynamicMap     - Dynamic occupancy map (starts empty)
%   config         - Struct with simulation parameters
%   robotMaps      - Cell array of local robot maps
%   globalMap      - Merged global map (initially empty)
%   objectLocations- Nx2 matrix of object coordinates
%   detectedObjects- Nx1 logical vector (detection flags)

    % Load example map from MATLAB exampleMaps.mat
    load exampleMaps.mat; % provides simpleMap
    refMap = binaryOccupancyMap(simpleMap, 1); % 1 meter resolution
    [mapdimx, mapdimy] = size(simpleMap);

    % Create an initially empty dynamic map (no obstacles marked)
    sampleMap2 = simpleMap;
    sampleMap2(sampleMap2 == 1) = 0;
    dynamicMap = binaryOccupancyMap(sampleMap2, 1);

    % Configuration
    config.numRobots       = 4;
    config.simulationTime  = 30;    % seconds
    config.sampleTime      = 0.1;   % seconds per control update
    config.stepSize        = 1;     % maximum step per levy walk update
    config.mapdimx         = mapdimx;
    config.mapdimy         = mapdimy;
    config.safeDistance    = 1;     % safe inter-robot distance (map units)
    config.detectionRadius = 2;     % object detection radius
    config.wallDistance    = 1;     % obstacle threshold for avoidance
    config.levyAlpha       = 5;     % Levy parameter alpha
    config.repulsionRange  = config.safeDistance * 3;

    % Robot local maps and global map (higher-resolution grid via occupancyMatrix)
    robotMaps = arrayfun(@(x) binaryOccupancyMap(mapdimx, mapdimy, 10), 1:config.numRobots, 'UniformOutput', false);
    globalMap = binaryOccupancyMap(mapdimx, mapdimy, 10);

    % Objects (example positions)
    objectLocations = [5, 5; 5, 20; 15, 20; 20, 5];
    detectedObjects = false(size(objectLocations, 1), 1);
end
