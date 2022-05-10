classdef LaserBot < UR3
    %LaserBot - A UR3 bot subclass with mounted camera and laser
    %   UR3 tasked with locating a QR Code target held by TargetBot and
    %   'shooting' the target with a laser pointer

    properties
        
        % Laser Parameters
        laserModel = 'Laser Model Here';
        laserRange = 3; %Placeholder (metres)
        laserOffset = zeros(4,4); % laser offset from end-effector

        targetPlots; % Simulation-fed target locations for 'GetImage'
        
        % Camera image
        cameraImage; % Property holding camera image 
        
        % Desired positions of target corners in camera plane based on
        % real target size
        targetSize = 0.2 % 20x20 cm
        targetDepth = 1; % Approximate distance between the bots
        cameraTargets;

        % Target data
        targetHit; % Binary property to mark if the target has been hit

    end

    methods
        function self = LaserBot()
            %LaserBot - LaserBot constructor, inheriting from UR3 baseclass
            %   Constructor calls base class
            self@UR3();
            self.name = 'LaserBot';
            self.model.base = eye(4,4); % Set the position of the model
            
            % Set up camera
            self.cameraOn = true;
            self.PlotCamera();
            
            % Move UR3 to start position
            self.MoveJoints(self.defaultPosition);
            
        end

        function PlotLaserBot(self)
            %PlotLaserBot - LaserBot Plotting for simulation
            % Need to overwrite UR3 model plot to include laser/camera in model
        end

        function SetCamera(self)
            % SetCamera - Sets target points in image plane based on target
            % size
            if self.cameraOn
                % Calculate object size in image, proprtional to 
                % realsize * focallength / objectdistance
                sizeInImage = self.targetSize*self.cameraModel.f/self.targetDepth;
                % Scale image to pixel size
                sizeInImage = sizeInImage/self.cameraModel.rho(1);
                % Set camera targets based on target size in image
                principalPoint = self.cameraModel.pp;
                u = [principalPoint(1)-sizeInImage/2, principalPoint(1)+sizeInImage/2];
                v = [principalPoint(2)-sizeInImage/2, principalPoint(2)+sizeInImage/2];
                self.cameraTargets = [u(2) u(1) u(1) u(2); v(1) v(1) v(2) v(2)];
                self.cameraModel.plot(self.cameraTargets, '*');
                self.cameraModel.hold;
            end
        end

        function GetImage(self)
            %GetImage - Retrieve image from camera
            Tc0 = self.model.fkine(self.model.getpos)*self.cameraOffset;
            self.cameraModel.plot(self.targetPlots, 'Tcam', Tc0, 'o');
        end

        function FindTarget(self)
            % FindTarget - Implements visual servoing to determine location of target from image and aim
            self.targetHit = false;
            self.GetImage();
            deltaT = 1/self.cameraFps;
            lambda = 0.6;
            i = 1;
            while ~self.targetHit 
                uv = self.cameraModel.plot(self.targetPlots);

                e = [self.cameraTargets(:,1)-uv(:,1);self.cameraTargets(:,2)-uv(:,2);self.cameraTargets(:,3)-uv(:,3);self.cameraTargets(:,4)-uv(:,4)];
            
                J = self.cameraModel.visjac_p(uv, self.targetDepth);
                v = lambda*pinv(J)*e;
                v = v*deltaT;
            
                nT = self.model.fkine(self.model.getpos())*transl(v(1),v(2),v(3))*trotx(v(4))*troty(v(5))*trotz(v(6));
                nq = self.model.ikcon(nT,self.model.getpos());
                self.model.animate(nq);
            
                Tc0 = self.model.fkine(self.model.getpos());
                self.cameraModel.T = Tc0;
            
                self.cameraModel.plot_camera('Tcam',Tc0,'scale',0.05);
                pause(deltaT);
                i = i+1;
                if max(e, [], 'all') < 15
                    display('Target reached!');
                    self.targetHit = true;
                elseif i >= 150
                    display('Target not reached!');
                    break
                end
            end
            self.cameraModel.clf();
        end

        function desiredPose = DetermineDesiredPose(self,targetLocation)
            % DetermineDesiredPose - Determine desired pose for laser
            % targeting

            
            return
                

        end

        function CheckIfHit(self)
            % CheckIfHit - Determine if target has been hit
        end

        function laserObject = ReturnLaser(self)
        %%Incomplete - Laser bot function that returns a laser object which
        %%can be used in robottargetsimulation for target bot with plane
        %%intersection to determine if there is a hit
            endEffectorTr = self.model.tool()
            [X,Y,Z] = cylinder([0,0.1],6);
            Z = Z * 10;
            updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';
            conePointsSize = size(X);
            cone_h = surf(reshape(updatedConePoints(:,1),conePointsSize) ...
             ,reshape(updatedConePoints(:,2),conePointsSize) ...
             ,reshape(updatedConePoints(:,3),conePointsSize));
            coneEnds = [cone_h.XData(2,:)', cone_h.YData(2,:)', cone_h.ZData(2,:)'];
            laserObject = endEffectorTr
            drawnow();
        end    
    end
end
