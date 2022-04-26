classdef TargetBot < UR3
    %TargetBot - A UR3 bot subclass with mounted camera and target
    %   UR3 tasked with holding a QR Code target to be 'shot' by LaserBot and
    %   moving the target to a random position

    properties
        
        % Camera Parameters
        cameraModel = 'Camera Model Here';
        
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

            self@UR3();
        end

        function PlotTargetBot(self,pos)
            %PlotTargetBot - TargetBot Plotting for simulation
            % Need to overwrite UR3 model plot to include target/camera in model
            if  hit == 1
            newq = self.ur3.model.ikine(pos);
            qMatrix = jtraj(self.ur3.model.getpos(),newq,self.steps);
            for i = 1:self.steps
                self.ur3.model.animate(qMatrix(i,:));
            end
            hit = 0;
            end
        
       
        end

        function NearSingularityM = CheckSingularity(self,q)
            J = self.ur3.model.jacob(q);
            m = sqrt(det(J*J))
            if(m > 0.1)
                NearSingularityM = 1;
            else
                NearSingularityM = 0;
            end
        end
    end
end