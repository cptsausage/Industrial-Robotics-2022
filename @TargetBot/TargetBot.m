classdef TargetBot < UR3
    %TargetBot - A UR3 bot subclass with mounted camera and target
    %   UR3 tasked with holding a QR Code target to be 'shot' by LaserBot and
    %   moving the target to a random position

    properties

        % Notes: do we nee a camera? Or should we just make a gui to move
        % it manually / set movement?
        % Camera Parameters

        % Target Parameters
        targetSize = 0.2; %Placeholder (metres)

        % Target Hit, if targetHit == 1, the target has been hit and will move to
        % new location

        targetHit;

        % Simulation-fed laser beam for line-plane intersection check
        laserPoints;
    end

    methods
        function self = TargetBot()
            %TargetBot - TargetBot constructor, inheriting from UR3 baseclass
            %   Constructor calls base class
            % Need to overwrite UR3 model plot to include target/camera in model

            self@UR3();
            self.name = 'TargetBot';
            self.model.base = transl(0,1.6,0)*trotz(-pi);
            self.MoveJoints(self.defaultPosition);
        end

        function hit = CheckIfHit(self)
            % CheckIfHit - function that loops and steps into 'MoveTarget' 
            % once TargetHit == 1 
            % Need to use some sort of indication, either through camera or
            % other sensor

            self.targetHit = 0;
            hit = 0;
            
            % Calculate normal to target plane for line plane intersection
            A = self.targetCorners(:,2)-self.targetCorners(:,1);
            B = self.targetCorners(:,3)-self.targetCorners(:,1);
            planeNormal = cross(A, B);

            [intersectionPoint,check] = LinePlaneIntersection(planeNormal,self.targetCorners(:,1),self.laserPoints(:,1),self.laserPoints(:,2));
            if check == 1
                % Check if point lies within target bounds, ignore y
                % because simulation target sits on plane
                xv = self.targetCorners(1,:);
                xz = self.targetCorners(3,:);
                if inpolygon(intersectionPoint(1), intersectionPoint(3), xv, xz)
                    hit = 1;
                    self.targetHit = 1;
                    display('TARGETBOT: Target hit!')
                else
                    display('TARGETBOT: Target not hit!')
                end
            else
                display('TARGETBOT: Target not hit!')
            end
        end

        function SetRandomTarget(self)
            % SetTarget - Set simulated target and target bot to random position within
            % set bounds
            display('TARGETBOT: Moving target...')
            xRange = [-0.2 0.2];
            yRange = [0.7 0.8];
            zRange = [0.4 0.6];
            x = xRange(1) + (xRange(2)-xRange(1))*rand(1,1);
            y = yRange(1) + (yRange(2)-yRange(1))*rand(1,1);
            z = zRange(1) + (zRange(2)-zRange(1))*rand(1,1);
            self.targetCorners = mkgrid(2, 0.1, 'T', transl(x,y-0.2,z)*trotx(pi/2)*trotz(-pi/2));
            q = self.model.ikcon(transl(x,y,z)*trotx(pi/2), self.model.getpos());
            self.MoveJoints(q);
        end
        
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

    end
end