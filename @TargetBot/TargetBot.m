classdef TargetBot < UR3
    %TargetBot - A UR3 bot subclass with mounted camera and target
    %   UR3 tasked with holding a QR Code target to be 'shot' by LaserBot and
    %   moving the target to a random position

    properties
        
        % Camera Parameters
        cameraModel = 'Camera Model Here';
        
        % Target Parameters
        targetSize = 0.1; %Placeholder (metres)

    end

    methods
        function self = TargetBot()
            %TargetBot - TargetBot constructor, inheriting from UR3 baseclass
            %   Constructor calls base class
            self@UR3();
        end

        function PlotTargetBot(self)
            %PlotTargetBot - TargetBot Plotting for simulation
            % Need to overwrite UR3 model plot to include target/camera in model
        end
    end
end