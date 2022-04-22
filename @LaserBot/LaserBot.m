classdef LaserBot < UR3
    %LaserBot - A UR3 bot subclass with mounted camera and laser
    %   UR3 tasked with locating a QR Code target held by TargetBot and
    %   'shooting' the target with a laser pointer

    properties
        
        % Camera Parameters
        cameraModel = 'Camera Model Here';
        
        % Laser Parameters
        laserModel = 'Laser Model Here';
        laserDistance = 3; %Placeholder (metres)

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
    end
end