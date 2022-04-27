classdef RoboTargeterSimulation
    %Simulation Simulation of robot interaction for code development
    %   Simulation of realworld environment and robot interaction before
    %   ROS networking

    properties
        
        % Properties holding target and laser robots
        laserBot;
        targetBot;

        % Simulated workspace size and parameters
        workspace;
        laserBotBase;
        targetBotBase;

    end

    methods
        function self = RoboTargeterSimulation()
            %RoboTargeterSimulation Initialisation of class object
            %   Creation of LaserBot and TargetBot objects
            self.laserBot = LaserBot();
            self.targetBot = TargetBot();
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end