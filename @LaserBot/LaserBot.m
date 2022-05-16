classdef LaserBot < UR3
    %LaserBot - A UR3 bot subclass with mounted camera and laser
    %   UR3 tasked with locating a QR Code target held by TargetBot and
    %   'shooting' the target with a laser pointer

    properties


        % Laser Parameters
        laserModel = 'Laser Model Here';
        laserRange = 3; %Placeholder (metres)
        laserOffset = zeros(4,4); % laser offset from end-effector
        laserPoints; % Points of laser beam to be passed through simulation

        targetPlots; % Simulation/Camera-fed target locations for 'GetImage'
        
        % Camera image
        cameraImage; % Property holding camera image 
        
        % Desired positions of target corners in camera plane based on
        % real target size
        targetSize = 0.18 % 18x18 cm
        targetDepth = 0.6; % Approximate distance between the target/laser
        cameraTargets;
        laserObject;
        % Target data
        targetAligned; % Binary property to mark if the target has been aligned
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
            
            % Move bot to default position
            % NOTE: NEED TO SKEW END EFFECTOR FOR CAMERA TO HELP WITH VISUAL SERVOIING!!!!!
            self.defaultPosition = deg2rad([45,-45,-45,-90,-45,135]);
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
                self.cameraTargets = [u(1) u(2) u(2) u(1); v(1) v(1) v(2) v(2)];
                if isempty(self.ur_cam)
                    % If simulating camera, plot the target points in the
                    % camera frame
                    self.cameraModel.plot(self.cameraTargets, '*');
                    self.cameraModel.hold;
                end
            end
        end

        function GetImage(self)
            %GetImage - Retrieve image from camera, otherwise get
            %simulation fed points
            try self.cameraImage = self.ur_cam.snapshot;
                pause(1/self.cameraFps);
            catch
                Tc0 = self.model.fkine(self.model.getpos)*self.cameraOffset;
                self.cameraModel.plot(self.targetPlots, 'Tcam', Tc0, 'o');
            end
        end

        function FindTarget(self)
            % FindTarget - Implements visual servoing to determine location of target from image and aim
            self.targetAligned = false;
            self.GetImage();
            deltaT = 1/self.cameraFps;
            lambda = 0.3;
            i = 1;
            display('LASERBOT: Beginning search...')
            while ~self.targetAligned
                if ~isempty(self.cameraImage)
                    % If the camera is sending images, perform real visual
                    % servoing
                    
                    refImage = imread('Target.png');
                    
                    % Find SIFTFeatures in image
                    refImage_gray = rgb2gray(refImage);
                    refImage_pts = detectSIFTFeatures(refImage_gray);
                    
                    % Detect SIFT features in real camera frame
                    
                    realImage = self.ur_cam.snapshot;
                    realImage_gray = rgb2gray(realImage);
                    realImage_pts = detectSIFTFeatures(realImage_gray);
                    
                    % Extract and Match Feature Points
                    [refFeatures, refPoints] = extractFeatures(refImage_gray, refImage_pts);
                    [realFeatures, realPoints] = extractFeatures(realImage_gray, realImage_pts);
                    
                    refPairs = matchFeatures(refFeatures, realFeatures);
                    
                    refPoints_matched = refPoints(refPairs(:,1), :);
                    realPoints_matched = realPoints(refPairs(:,2),:);
                    
                    % Locate Object in Scene using Feature Matches
                    try

                        [tform, refPoints_inlier, realPoints_inlier] = ...
                            estimateGeometricTransform(refPoints_matched, realPoints_matched, "affine");
                        
                        % Display Location in Real Image
                        
                        boxPolygon = [1, 1;...
                            size(refImage_gray,2), 1;...
                            size(refImage_gray,2), size(refImage_gray, 1);...
                            1, size(refImage_gray, 1);...
                            1, 1];
                        
                        % Transform polygon onto new image
                        
                        newBoxPolygon = transformPointsForward(tform, boxPolygon);
                        figure(2);
                        imshow(realImage);
                        hold on
                        line(newBoxPolygon(:,1),newBoxPolygon(:,2),'Color','y','LineWidth',5);
                        hold off;
                        title('UR Camera View');

                        % Calculate Errors
                        uv = newBoxPolygon(1:4,:)';
    
                        e = [self.cameraTargets(:,1)-uv(:,1);self.cameraTargets(:,2)-uv(:,2);self.cameraTargets(:,3)-uv(:,3);self.cameraTargets(:,4)-uv(:,4)];
                
                        targetBox = [self.cameraTargets, self.cameraTargets(:,1)];
                        if max(abs(e), [], 'all') < 20
                            figure(2);
                            line(targetBox(1,:), targetBox(2,:),'Color','g','LineWidth',5)
                            
                            display('LASERBOT: Target aligned!');
                        
                            % FIRE THE LASERS!!!
                            figure(1);
                            laserStart = self.model.fkine(self.model.getpos());
                            laserEnd = laserStart*transl(0,0,self.laserRange);
                            self.laserPoints = [laserStart(1:3,4), laserEnd(1:3,4)];
                            laserPlot = line(self.laserPoints(1,:), self.laserPoints(2,:), self.laserPoints(3,:), 'Color', 'r', 'LineWidth', 5);
                            pause(1);
                            delete(laserPlot);
                            
                            self.targetAligned = true;
                            break
                        elseif i >= 150
                            display('LASERBOT: Target not aligned within step limit!');
                            break
                        else
                            figure(2);
                            line(targetBox(1,:), targetBox(2,:),'Color','r','LineWidth',5);
                        end

                        J = self.cameraModel.visjac_p(uv, self.targetDepth);
                        lambda = 0.8;                   % Increase gain for ros interaction
                        v = lambda*pinv(J)*e;
                    
                        %compute robot's Jacobian and inverse
                        J2 = self.model.jacobn(self.model.getpos);
                        Jinv = pinv(J2);
                        % get joint velocities
                        qv = Jinv*v;
    
                        figure(1);
                        hold on
                        deltaT = 1/4;
                        nq = self.model.getpos()' + deltaT*qv; % Alter deltaT to account for ros message sending
                        self.model.animate(nq');
                    
                        Tc0 = self.model.fkine(self.model.getpos());
                        self.cameraModel.T = Tc0;
                        hold off

                        if self.ROSOn == 1 
                            display('ROS is on...')
                            % Establish Joint Names
                            jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
                            pause(0.01);
                            currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position); % Note the default order of the joints is 3,2,1,4,5,6
                            currentJointState_123456 = [currentJointState_321456(3:-1:1)',currentJointState_321456(4:6)'];
            
                            
                            jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
                
                            [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
                            goal.Trajectory.JointNames = jointNames;
                            goal.Trajectory.Header.Seq = 1;
                            goal.Trajectory.Header.Stamp = rostime('Now','system');
                            goal.GoalTimeTolerance = rosduration(0.05);
                            bufferSeconds = 0.5; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
                            durationSeconds = 1; % This is how many seconds the movement will take
            
                            % Get current joint state
                            startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                            startJointSend.Positions = currentJointState_123456;
                            startJointSend.TimeFromStart = rosduration(0);     
                              
                            % Get next/final joint state
                            endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                            endJointSend.Positions = nq';
                            
                            endJointSend.TimeFromStart = rosduration(durationSeconds);
                            
                            % Set goal
                            goal.Trajectory.Points = [startJointSend; endJointSend];
            
                            % Send goal to UR3
                            goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
                            waitForServer(client);
                            sendGoal(client,goal);
                            pause(durationSeconds + bufferSeconds);
                        end
                        
                        pause(deltaT);
                        i = i+1;

                    catch
                        % If not enough matching SIFT features are found,
                        % go through search routine
                        figure(2);
                        imshow(realImage);
                    end
                else
                    % Simulation plotting if camera is not set up, use
                    % virtual visual servoing
                    uv = self.cameraModel.plot(self.targetPlots);
    
                    e = [self.cameraTargets(:,1)-uv(:,1);self.cameraTargets(:,2)-uv(:,2);self.cameraTargets(:,3)-uv(:,3);self.cameraTargets(:,4)-uv(:,4)];
                
                    J = self.cameraModel.visjac_p(uv, self.targetDepth);
                    v = lambda*pinv(J)*e;
                
                    %compute robot's Jacobian and inverse
                    J2 = self.model.jacobn(self.model.getpos);
                    Jinv = pinv(J2);
                    % get joint velocities
                    qv = Jinv*v;

                    figure(1);
                    nq = self.model.getpos()' + deltaT*qv; % Alter deltaT to account for ros message sending
                    self.model.animate(nq');
            
                    Tc0 = self.model.fkine(self.model.getpos());
                    self.cameraModel.T = Tc0;
                
                    self.cameraModel.plot_camera('Tcam',Tc0,'scale',0.05);
    %                 display('Test -2')
                    if self.ROSOn == 1 
                        self.ROSSendGoal(nq);
    %                     display('Test 0')
                        pause(2); % For testing
                    end
    %                 display('Test -1')
                    pause(deltaT);
                    i = i+1;
                    if max(abs(e), [], 'all') < 20
                        display('Target reached!');
                        
                        % FIRE THE LASERS!!!
                        laserStart = self.model.fkine(self.model.getpos());
                        laserEnd = laserStart*transl(0,0,self.laserRange);
                        self.laserPoints = [laserStart(1:3,4), laserEnd(1:3,4)];
                        laserPlot = line(self.laserPoints(1,:), self.laserPoints(2,:), self.laserPoints(3,:), 'Color', 'r', 'LineWidth', 5);
                        pause(1);
                        delete(laserPlot);
                        
                        self.targetAligned = true;
                    elseif i >= 150
                        display('Target not reached!');
                        break
                    end
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

        function ReturnLaser(self,q)
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
            
            drawnow();
        end

        function MoveJointsWithLaser(self,q)    
            
            deltaT = 1/self.cameraFps;
            steps = 100;
            qMatrix = jtraj(self.model.getpos(),q,steps);
            for i = 1:steps
            hold on
            if(i ~= 1)
                delete(cone_h)
            end
            endEffectorTr = self.model.getpos()
            eT = self.model.fkine(endEffectorTr)
    
            [X,Y,Z] = cylinder([0,0.1],6);
            Z = Z * 10;
            updatedConePoints = [eT * [X(:),Y(:),Z(:),ones(numel(X),1)]']';
            conePointsSize = size(X);
            cone_h = surf(reshape(updatedConePoints(:,1),conePointsSize) ...
             ,reshape(updatedConePoints(:,2),conePointsSize) ...
             ,reshape(updatedConePoints(:,3),conePointsSize));
            coneEnds = [cone_h.XData(2,:)', cone_h.YData(2,:)', cone_h.ZData(2,:)'];
              
                for j = 1:self.model.n                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        display(['Reaching joint ' num2str(j) ' limit, stopping...'])
                        return
                    elseif qMatrix(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        display(['Reaching joint ' num2str(j) ' limit, stopping...'])
                        return
                    end
                end
                self.model.animate(qMatrix(i,:));
                if self.cameraOn == true
                    self.cameraModel.T = self.model.fkine(self.model.getpos())*self.cameraOffset; % Update camera position
                    self.cameraModel.plot_camera();
                end
                pause(deltaT);
                hold on
            end
        end

    end
end
