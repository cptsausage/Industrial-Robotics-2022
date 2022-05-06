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
        
        % Desired positions of target corners in camera plane
        cameraTargets = [...
            662 362 362 662;...
            362 362 662 662]; 

        % Target data
        targetHit; % Binary property to mark if the target has been hit

    end

    methods
        function self = LaserBot()
            %LaserBot - LaserBot constructor, inheriting from UR3 baseclass
            %   Constructor calls base class
            self@UR3();
            self.name = 'LaserBot';
            % Move UR3 to start position
            self.MoveJoints(self.defaultPosition);
            
        end

        function PlotLaserBot(self)
            %PlotLaserBot - LaserBot Plotting for simulation
            % Need to overwrite UR3 model plot to include laser/camera in model
        end

        function SetCamera(self)
            % SetCamera - Sets target points in image plane
            if self.cameraOn
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
            depth = 1;
            lambda = 5;
            i = 1;
            while ~self.targetHit
                    uv = self.cameraModel.plot(self.targetPlots);

                    e = [self.cameraTargets(:,1)-uv(:,1);self.cameraTargets(:,2)-uv(:,2);self.cameraTargets(:,3)-uv(:,3);self.cameraTargets(:,4)-uv(:,4)];
                
                    J = self.cameraModel.visjac_p(uv, depth);
                    velocity(i,:) = lambda*pinv(J)*e;
                    v = velocity(i,:)*deltaT;
                
                    nT = self.model.fkine(self.model.getpos())*transl(v(1),v(2),v(3))*trotx(v(4))*troty(v(5))*trotz(v(6));
                    nq = self.model.ikcon(nT,self.model.getpos());
                    self.model.animate(nq);
                
                    Tc0 = self.model.fkine(self.model.getpos());
                    self.cameraModel.T = Tc0;
                
                    self.cameraModel.plot_camera('Tcam',Tc0,'scale',0.15);
                    pause(deltaT);
                    i = i+1;
                    if max(e, [], 'all') < 20
                        display('Target reached!');
                        self.targetHit = true;
                    end
            end
        end

        function desiredPose = DetermineDesiredPose(self,targetLocation)
            % DetermineDesiredPose - Determine desired pose for laser
            % targeting

            
            return
                

        end

        function CheckIfHit(self)
            % CheckIfHit - Determine if target has been hit
        end

    end
end
