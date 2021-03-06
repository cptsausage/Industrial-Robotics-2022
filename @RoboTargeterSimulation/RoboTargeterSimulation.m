 classdef RoboTargeterSimulation
    %Simulation Simulation of robot interaction for code development
    %   Simulation of realworld environment and robot interaction before
    %   ROS networking

    properties
        
        % Properties holding target and laser robots
        laserBot;
        targetBot;
        laser;
        hazard;
        gg;

        % Simulated workspace size and parameters
        workspace;
        targetsHit = 0;
        time; % seconds
        stop = 0; % Simulation stop

    end

    methods
        
        function self = RoboTargeterSimulation(~)
            %RoboTargeterSimulation Initialisation of class object
            %   Creation of LaserBot and TargetBot objects
            close all;
            set(0,'DefaultFigureWindowStyle','normal')

            hold("on");
            
            % Load in the environment floors
            surf([-0.5,-0.5;0.5,0.5],[-0.5,1.5;-0.5,1.5],[0.01,0.01;0.01,0.01], ...
               'CData',imread('wood.jpg'),'FaceColor','texturemap');
            surf([-2,-2;2,2],[-2,2;-2,2],[-0.75,-0.75;-0.75,-0.75], ...
               'CData',imread('marble.jpg'),'FaceColor','texturemap');

            % Load in the environment walls
            surf([2,-2;2,-2],[-2,-2;-2,-2],[2,2;-0.75,-0.75], ...
               'CData',imread('brick-wall.jpg'),'FaceColor','texturemap');
            surf([1,-1;1,-1],[-1.9,-1.9;-1.9,-1.9],[1.75,1.75;0.75,0.75], ...
               'CData',imread('laser_warning.jpg'),'FaceColor','texturemap');
            
            % Load in the table model
            [f,v,data] = plyread('new_table_4.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1),v(:,2)+0.5, v(:,3)-0.06 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            % Load in the safety features
            [f,v,data] = plyread('dennis.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            dennis_h = trisurf(f,v(:,1), v(:,2)-1.3, v(:,3)-0.7 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            [f,v,data] = plyread('estop.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            estop_h = trisurf(f,v(:,1)-0.65, v(:,2)-0.35, v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            % Load in the safety features (light poles)
            [f,v,data] = plyread('light_pole.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            lightPole1_h = trisurf(f,v(:,1)+1.1, v(:,2)+1.7, v(:,3)-0.75 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            [f,v,data] = plyread('light_pole.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            lightPole2_h = trisurf(f,v(:,1)-1.1, v(:,2)+1.7, v(:,3)-0.75 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            [f,v,data] = plyread('light_pole.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            lightPole3_h = trisurf(f,v(:,1)+1.1, v(:,2)-0.75, v(:,3)-0.75 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            [f,v,data] = plyread('light_pole.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            lightPole4_h = trisurf(f,v(:,1)-1.1, v(:,2)-0.75, v(:,3)-0.75 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            self.gg = gourgeousGavin()
            [f,v,data] = plyread('dennis.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.gg.gavinModel = trisurf(f,v(:,1), v(:,2)-1.3, v(:,3)-0.7 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            self.laserBot = LaserBot();
            hold on;
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

        function CreateHazard(self)
            % Change size and position here
            figure(1);
            hold on
            size = 0.2;
            x = -0.1;
            y = 0.7;
            z = 0.4;
            self.laserBot.hazardPlots = [...
                x-size/2, x+size/2, x+size/2, x-size/2, x-size/2;...
                y, y, y, y, y;...
                z+size/2, z+size/2, z-size/2, z-size/2, z+size/2];
            self.laserBot.hazard = surf([x-size/2, x+size/2; x-size/2, x+size/2,],...
                [y, y; y, y],...
                [z+size/2, z+size/2; z-size/2, z-size/2],...
                'CData',imread('hazardsign.jpg'),'FaceColor','texturemap');
            hold off
        end

        function RemoveHazard(self,~)
            self.laserBot.hazardPlots = [];
            delete(self.laserBot.hazard);
        end

        function STOP(self)
            while self.stop == 1
                pause(1);
            end
        end

        function CreateObstacle(self)
            % Change size and position here
            figure(1);
            hold on
            size = 0.2;
            x = -0.1;
            y = 0.7;
            z = 0.4;
            % Obstacle corners
            % Spawn Cube

            % Spawn the obstacle model
            [f,v,data] = plyread('obstacle.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.laserBot.obstacle = trisurf(f,v(:,1),v(:,2)+0.5, v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','flat');
            hold off
        end

        function MoveDennis(self,~)
            self.gg.gavinModel
            hold on
            
            delete(self.gg.gavinModel)
            
            isvalid(self.gg.gavinModel)
            isgraphics(self.gg.gavinModel)

            decision = round(rand())
%             if decision == 0
%                 [f,v,data] = plyread('dennis.ply','tri');
%                 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%                 self.gg.gavinModel = trisurf(f,v(:,1), v(:,2)-1.3, v(:,3)-0.7 ...
%                     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%             end
            if decision == 1
                [f,v,data] = plyread('dennis.ply','tri');
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                self.gg.gavinModel = trisurf(f,v(:,1), v(:,2)-0.8, v(:,3)-0.7 ...
                    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
            self.gg.gavinModel
            hold off
        end

        function RemoveObstacle(self,~)
            % Delete the ellipsoid
            self.laserBot.obstaclePlots = [];

            % Delete the model
            delete(self.laserBot.obstacle);
        end
    
    end
end