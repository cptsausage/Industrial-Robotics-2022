%% TargetBot Standalone Code
close all;
clear;

% Initialise LaserBot
targetBot = TargetBot();

% Initialise ROS connection
rosshutdown;
rosinit('192.168.0.253'); % UTS Pi address
laserBot.ROSInit();

targetBot.SetRandomTarget();