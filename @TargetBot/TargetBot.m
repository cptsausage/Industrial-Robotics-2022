classdef TargetBot < UR3
    %TargetBot - A UR3 bot subclass with mounted camera and target
    %   UR3 tasked with holding a QR Code target to be 'shot' by LaserBot and
    %   moving the target to a random position

    properties

        % Notes: do we nee a camera? Or should we just make a gui to move
        % it manually / set movement?
        % Camera Parameters
        
        % Target
        targetCorners;
        targetPlot;

        % Target Parameters
        targetSize = 0.1; %Placeholder (metres)

        % Target Hit, if hit == 1, the target has been hit and will move to
        % new location

        hit = 0;
    end

    methods
        function self = TargetBot()
            %TargetBot - TargetBot constructor, inheriting from UR3 baseclass
            %   Constructor calls base class
            % Need to overwrite UR3 model plot to include target/camera in model

            self@UR3();
            self.name = 'TargetBot';
        end

        function MoveTarget(self)
            % MoveTarget - Moving the target to a new location
            % Use main class UR3 MoveBot for main movement/singularity/optimisation and
            % collision detection
        end

        function CheckIfHit(self)
            % CheckIfHit - function that loops and steps into 'MoveTarget' 
            % once TargetHit == 1 
            % Need to use some sort of indication, either through camera or
            % other sensor
        end

        function SetRandomTarget(self)
            % SetTarget - Random Target setting for simulation
            xRange = [1.3 1.5];
            yRange = [-0.3 0.3];
            zRange = [0.2 0.4];
            x = xRange(1) + (xRange(2)-xRange(1))*rand(1,1);
            y = yRange(1) + (yRange(2)-yRange(1))*rand(1,1);
            z = zRange(1) + (zRange(2)-zRange(1))*rand(1,1);
            self.targetCorners = mkgrid(2, 0.5, 'T', transl(x,y,z)*troty(pi/2));
            self.PlotTarget;
        end

        function PlotTarget(self)
            clear self.targetPlot
            hold on
            plot_sphere(self.targetCorners,0.05,'b');
        end
% MIGHT SPLIT 'PlotTargetBot' INTO 'MoveTarget' AND 'MoveBot' IN UR3 MAIN
% CLASS
        
        function PlotTargetBot(self,pos)
            %PlotTargetBot - TargetBot Plotting for simulation
            if  hit == 1
            newq = self.ur3.model.ikine(pos);
            qMatrix = jtraj(self.ur3.model.getpos(),newq,self.steps);
            for i = 1:self.steps
                self.ur3.model.animate(qMatrix(i,:));
            end
            hit = 0;
            end
        
       
        end

% MOVING 'NearSingularityM' TO UR3 MAIN CLASS TO BE INHERITED BY BOTH BOTS

%         function NearSingularityM = CheckSingularity(self,q)
%             J = self.ur3.model.jacob(q);
%             m = sqrt(det(J*J))
%             if(m > 0.1)
%                 NearSingularityM = 1;
%             else
%                 NearSingularityM = 0;
%             end
%         end
    end
end