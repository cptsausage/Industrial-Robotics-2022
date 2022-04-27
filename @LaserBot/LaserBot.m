classdef LaserBot < UR3
    %LaserBot - A UR3 bot subclass with mounted camera and laser
    %   UR3 tasked with locating a QR Code target held by TargetBot and
    %   'shooting' the target with a laser pointer

    properties
        
        % Camera Parameters
        cameraModel = 'Camera Model Here';
        cameraOffset = zeros(4,4); % camera offset from end-effector
        
        % Laser Parameters
        laserModel = 'Laser Model Here';
        laserRange = 3; %Placeholder (metres)
        laserOffset = zeros(4,4); % laser offset from end-effector

        % Camera image
        cameraImage; % Property holding camera image 

        % Target data
        targetHit = 0; % Binary property to mark if the target has been hit
        targetPose; % Output target pose from endeffector
        targetDistance; % Output of target distance from laserbot

    end

    methods
        function self = LaserBot()
            %LaserBot - LaserBot constructor, inheriting from UR3 baseclass
            %   Constructor calls base class
            self@UR3();
        end

        function PlotLaserBot(self)
            %PlotLaserBot - LaserBot Plotting for simulation
            % Need to overwrite UR3 model plot to include laser/camera in model
        end

        function GetImage(self)
            %GetImage - Retrieve image from camera
        end

        function FindTarget(self)
            % FindTarget - Determine location of target from image
        end

        function DetermineDesiredPose(self)
            % DetermineDesiredPose - Determine desired pose for laser
            % targeting
        end

        function CheckIfHit(self)
            % CheckIfHit - Determine if target has been hit
        end

    end
end
