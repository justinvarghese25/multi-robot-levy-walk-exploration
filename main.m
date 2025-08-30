% main.m
% Entry point for multi-robot Lévy walk exploration simulation.

clc; clear; close all;

% Initialize environment, maps, and parameters
[refMap, dynamicMap, config, robotMaps, globalMap, objectLocations, detectedObjects] = initializeEnvironment();

% Initialize robot poses, controllers, sensors, kinematics
[robotPoses, controllers, sensors, robotKinematics] = initializeRobots(config.numRobots, refMap);

% Run simulation loop
[robotTrajectories, globalMap, V_total] = runSimulation(config, refMap, dynamicMap, ...
    robotMaps, globalMap, objectLocations, detectedObjects, robotPoses, controllers, sensors, robotKinematics);

% Plot final results
plotResults(refMap, robotTrajectories, globalMap, objectLocations, detectedObjects, V_total);
