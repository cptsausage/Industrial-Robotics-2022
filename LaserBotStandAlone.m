%% LaserBot Standalone Code
close all;
clear;

% Initialise LaserBot
laserBot = LaserBot();

% Initialise Camera
laserBot.CameraInit();
laserBot.SetCamera(); % To set cameraTargets for error calculation

% Initialise ROS connection
% rosshutdown;
% rosinit('192.168.0.253'); % UTS Pi address
% laserBot.ROSInit();

laserBot.FindTarget();

