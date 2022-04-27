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
        targetsHit = 0;
        time; % seconds

    end

    methods
        function self = RoboTargeterSimulation()
            %RoboTargeterSimulation Initialisation of class object
            %   Creation of LaserBot and TargetBot objects
            self.laserBot = LaserBot();
            self.targetBot = TargetBot();
        end

        function InitialiseSimulation(self)
            %InitialiseSimulation - Move bots to base positions and begin
            %timer
        end

        function CalculateError(self)
            %CalculateError - Calculate error/accuracy between target and
            %laser (using line-plane intersection and target center)
        end
    end
end