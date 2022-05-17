 classdef RoboTargeterSimulation
    %Simulation Simulation of robot interaction for code development
    %   Simulation of realworld environment and robot interaction before
    %   ROS networking

    properties
        
        % Properties holding target and laser robots
        laserBot;
        targetBot;
        laser;

        % Simulated workspace size and parameters
        workspace;
        targetsHit = 0;
        time; % seconds

    end

    methods
        function self = RoboTargeterSimulation(~)
            %RoboTargeterSimulation Initialisation of class object
            %   Creation of LaserBot and TargetBot objects
            self.workspace = [-0.6 0.6 0 1.4 -0.3 1];
            
            surf([0.6, 0.6; -0.6, -0.6],[1.4, -0.2; 1.4, -0.2],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            hold('on');
            surf([-0.6,0.6;-0.6,0.6],[-0.2,-0.2;-0.2,-0.2],[1,1;0.01,0.01],[1,1;0.01,0.01],'CData',imread('TargetsImage.jpg'),'FaceColor','texturemap');
            surf([-0.6,-0.6;-0.6,-0.6],[-0.2,1.4;-0.2,1.4],[1,1;0.01,0.01],'CData',imread('TargetsImage.jpg'),'FaceColor','texturemap');
            
            self.laserBot = LaserBot();
            self.targetBot = TargetBot();
        end

        function InitialiseSimulation(self)
            %InitialiseSimulation - Move bots to base positions and begin
            %timer
        end

        function RandomTargetPractice(self)
            % RandomTargetPractice - Uses bot functions to randomly
            % generate floating targets for the laser bot to find and
            % target

%             self.laserBot.ROSInit();
            
            hits = 0;

            for round = 1:5

                self.targetBot.SetRandomTarget();
                
    
                % For simulation, give target positions to laserbot for
                % 'GetImage' function
                self.laserBot.targetPlots = self.targetBot.targetCorners;
    
                self.laserBot.SetCamera();
    
                self.laserBot.FindTarget();

                if self.laserBot.targetAligned
                    % Pass through the laser points to check intersection
                    % with target
                    display('LASERBOT: Target Aligned...')
                    self.targetBot.laserPoints = self.laserBot.laserPoints;
                    if self.targetBot.CheckIfHit
                        hits = hits + 1;
                    end
                end
            
            end
            display([num2str(round), ' rounds over, final score is ', num2str(hits), ' hits!'])

        end

        function hazard = CreateHazard(self)
            % Change size and position here
            size = 0.2;
            x = -0.1;
            y = 0.7;
            z = 0.4;
            self.laserBot.hazardPlots = [...
                x-size/2, x+size/2, x+size/2, x-size/2, x-size/2;...
                y, y, y, y, y;...
                z+size/2, z+size/2, z-size/2, z-size/2, z+size/2];
            hazard = surf(...
                [x-size/2, x+size/2; x-size/2, x+size/2,],...
                [y, y; y, y],...
                [z+size/2, z+size/2; z-size/2, z-size/2],...
                'CData',imread('hazardsign.jpg'),'FaceColor','texturemap');
        end

        function RemoveHazard(self, hazard)
            delete(self.laserBot.hazardPlots)
            delete(hazard)
        end

        function CalculateError(self)
            %CalculateError - Calculate error/accuracy between target and
            %laser (using line-plane intersection and target center)
        end
    end
end