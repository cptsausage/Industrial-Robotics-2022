classdef UR3 < handle
    %UR3 UR3 model class
    %   Contains main properties and methods shared between LaserBot and
    %   TargetBot

    properties
        
        name = 'UR3';
        
        % Using model property for UR3 serial links
        model;

        % Designated UR3 workspace for initial plot
        workspace = [-2 2 -2 2 -0.8 2];

        % Default starting position for UR3
        defaultPosition = deg2rad([0,-45,-90,-45,0,0]);

        % Nice starting pose before performing RMRC in cartesian plane
        startT = transl(0.4,0,0.5)*troty(pi/2);

        % Camera Parameters
        cameraOn = false;
        % CHANGE ONCE USB CAM SPECS ARE FOUND
        ur_cam;                                 % Property for webcam
        cameraModel = CentralCamera(...
                'focal', 0.001,...
                'pixel', 2e-6,...
                'resolution', [640 480],...
                'centre', [320 240], ...
                'name', 'C922 Pro Stream Webcam');
        cameraOffset = transl(0,0,0); % camera offset from end-effector
        cameraFps = 60; 

        % Target plot for target on ur3
        targetCorners;
        targetPlot;

        % Boolean to communicate to UR3 through ROS
        ROSOn = false;

        % Boolean to run analysis of RMRC
        analysis = false;

    end

    methods
        function self = UR3()
            % UR3 - Main constructor initialising UR3 object
            
            self.GetUR3Robot();         % Create UR3 SerialLink model
            
            self.PlotUR3();             % Plot UR3 in workspace with model mesh

            drawnow

        end

        function GetUR3Robot(self)
            %GetUR3Robot - Creates UR3 bot using SerialLink with D&H
            %parameters
            
            pause(0.001);
            self.name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];


            % D&H parameters from universal robotics website: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
            % Joint limits found in UR3 user manual: https://s3-eu-west-1.amazonaws.com/ur-support-site/32341/UR3_User_Manual_en_E67ON_Global-3.5.5.pdf

            L(1) = Link([0      0.1519  0           -pi/2   0]); 
            L(2) = Link([0      0.120   0.24365     0       0]);
            L(3) = Link([0      -0.093  0.21325     0       0]);
            L(4) = Link([0      0.11235 0           -pi/2   0]);
            L(5) = Link([0      0.08535 0           pi/2    0]);
            L(6) = Link([0      0.0819  0           0	    0]);
    
            % Incorporate joint limits
            L(1).qlim = [-360 360]*pi/180;
            L(2).qlim = [-90 90]*pi/180;
            L(3).qlim = [-170 170]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;

            self.model = SerialLink(L,'name',self.name);

        end

        function PlotUR3(self)
            % PlotUR3 - Plots the UR3 robot with model

            for linkIndex = 0:self.model.n
                
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
        
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
        
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end

        function CameraInit(self)
            self.ur_cam = webcam(self.cameraModel.name);
        end

        function PlotCamera(self)
            % If camera setting is on, create and plot camera object onto
            % model
            if self.cameraOn
                Tc0 = self.model.fkine(self.model.getpos)*self.cameraOffset;
                self.cameraModel.T = Tc0;
                self.cameraModel.plot_camera('Tcam',Tc0,'scale',0.05);
            end

        end

        function PlotTarget(self)
            % PlotTarget - Plots simulated target square to UR3 end
            % effector
            if ~isempty(self.targetCorners)
                T = self.model.fkine(self.model.getpos());
                T = T(1:3,4)';
                self.targetCorners = mkgrid(2, 0.1, 'T', transl(T(1),T(2)-0.01,T(3))*trotx(pi/2)*trotz(-pi/2)); % Target has slight offset to avoid clipping into ur3 model
                delete(self.targetPlot);
                hold on;
                plot = [self.targetCorners(1,1),self.targetCorners(1,2);...
                    self.targetCorners(1,4), self.targetCorners(1,3);...
                    self.targetCorners(2,1), self.targetCorners(2,2);...
                    self.targetCorners(2,4), self.targetCorners(2,3);...
                    self.targetCorners(3,1), self.targetCorners(3,2);...
                    self.targetCorners(3,4), self.targetCorners(3,3)];
                self.targetPlot = surf(plot(1:2,:), plot(3:4,:), plot(5:6,:),'CData',imread('Target.png'),'FaceColor','texturemap');
            end
        end

        function ROSInit(self)
            
            jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
            pause(2);

            if ~isempty(jointStateSubscriber.LatestMessage)
                display('ROS CONNECTED')
                self.ROSOn = true;
            else
                display ('ROS NOT CONNECTED')
            end

            if self.ROSOn
                jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
                pause(2); % Pause to give time for a message to appear
                currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position); % Note the default order of the joints is 3,2,1,4,5,6
                startq = [currentJointState_321456(3:-1:1)',currentJointState_321456(4:6)'];
                self.model.animate(startq);
            end

            self.MoveJoints(self.defaultPosition);
             
        end

        function ROSSendGoal(self, q)
            % Check if ROS communication is enabled
            if self.ROSOn == true
                display('ROS is on...')
                % Establish Joint Names
                jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
                pause(1);
                currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position); % Note the default order of the joints is 3,2,1,4,5,6
                currentJointState_123456 = [currentJointState_321456(3:-1:1)',currentJointState_321456(4:6)'];

                
                jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
    
                [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
                goal.Trajectory.JointNames = jointNames;
                goal.Trajectory.Header.Seq = 1;
                goal.Trajectory.Header.Stamp = rostime('Now','system');
                goal.GoalTimeTolerance = rosduration(0.05);
                bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
                durationSeconds = 4; % This is how many seconds the movement will take

                % Get current joint state
                startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                startJointSend.Positions = currentJointState_123456;
                startJointSend.TimeFromStart = rosduration(0);     
                  
                % Get next/final joint state
                endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                endJointSend.Positions = q;
                
                endJointSend.TimeFromStart = rosduration(durationSeconds);
                
                % Set goal
                goal.Trajectory.Points = [startJointSend; endJointSend];

                % Send goal to UR3
                goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
                waitForServer(client);
                sendGoal(client,goal);
                pause(durationSeconds+bufferSeconds*2);
            end
        end

        function MoveJoints (self, q)
            % MoveJoints - For joint interpolation movement
            deltaT = 1/self.cameraFps;
            steps = 100;
            qMatrix = jtraj(self.model.getpos(),q,steps);
            for i = 1:steps
                if self.CheckCollisions(qMatrix(i,:))
                    display('WARNING, COLLISION DETECTED FROM JOINT INTERPOLATION MOVEMENT. STOPPING MOVEMENT.')
                    return
                end
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
                self.PlotCamera();
                self.PlotTarget();
                pause(deltaT);

            end
            if self.ROSOn == 1 
                self.ROSSendGoal(qMatrix(i,:));
            end
        end

        function MoveBot(self, desiredPose)
            % MoveBot - Main function for moving the robot using RMRC

            % RMRC PARAMETERS
            t = 10;              % Allcoated time (s) for movement, need to make dynamic for small/large movements
            deltaT = 1/self.cameraFps;
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold for DLS optimisation
            W = diag([1 1 1 0.1 0.1 0.1]);    % Velocity vector weighting matrix

            % ARRAY ALLOCATION
            m = zeros(steps,1);                     % Array for Measure of Manipulability
            qMatrix = zeros(steps,self.model.n);    % Array for joint angles
            qdot = zeros(steps,self.model.n);       % Array for joint velocities
            theta = zeros(3,steps);                 % Array for roll-pitch-yaw angles
            x = zeros(3,steps);                     % Array for x-y-z trajectory
            truePos = zeros(3,steps);               % Array for plotting true x-y-z trajectory
            positionError = zeros(3,steps);         % For plotting trajectory error
            angleError = zeros(3,steps);            % For plotting trajectory error

            % Get start and end x-y-z/roll pitch yaw
            qMatrix(1,:) = self.model.getpos();     % Get initial joint state
            startPose = self.model.fkine(qMatrix(1,:));
            startX = startPose(1:3,4)';
            endX = desiredPose(1:3,4)';
            startTheta = tr2rpy(startPose);
            endTheta = tr2rpy(desiredPose);
            
            % Straight line trajectory planning
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = startX(1) + s(i)*((endX(1)-startX(1)));         % Points in x
                x(2,i) = startX(2) + s(i)*((endX(2)-startX(2)));         % Points in y
                x(3,i) = startX(3) + s(i)*((endX(3)-startX(3)));         % Points in z
                theta(1,i) = startTheta(1) + s(i)*((endTheta(1)-startTheta(1)));     % Roll angle 
                theta(2,i) = startTheta(1) + s(i)*((endTheta(1)-startTheta(1)));     % Pitch angle
                theta(3,i) = startTheta(1) + s(i)*((endTheta(1)-startTheta(1)));     % Yaw angle
            end
            hold on
            traj_h = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1);

            % Use RMRC to move the robot along the trajectory
            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = self.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:self.model.n                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                        display(['Reaching joint ' num2str(j) ' limit'])
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                        display(['Reaching joint ' num2str(j) ' limit'])
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                
                if self.CheckCollisions(qMatrix(i+1,:))                                   % Check for collision in next joint step
                    display('WARNING, COLLISION DETECTED FROM RMRC MOVE BOT. STOPPING MOVEMENT.')
                    return
                end
                truePos(:,i) = T(1:3,4);
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting

                self.model.animate(qMatrix(i,:));
                self.targetPlot();
                self.PlotCamera();
                self.ROSSendGoal(qMatrix(i,:));
                path_h = plot3(truePos(1,:),truePos(2,:),truePos(3,:),'r.','LineWidth',1);
                pause(deltaT);
            end

            display(['Final position error: ', num2str(positionError(:,steps-1)')])
            display(['Final angle error: ', num2str(angleError(:,steps-1)')])

            % Plot RMRC results for analysis
            if self.analysis == true
                self.RMRCResults(qMatrix, qdot, positionError, angleError, m, epsilon);
            end

        end

        function JointSet(self, ~, input)
            % JointSet - Function called during event where GUI joint values are updated
            
            % Get original joints
            q0 = self.model.getpos();

            % Set all joints from GUI input
            q(1) = q0(1) + input(1)*(pi/180);
            q(2) = q0(2) + input(2)*(pi/180);
            q(3) = q0(3) + input(3)*(pi/180);
            q(4) = q0(4) + input(4)*(pi/180);
            q(5) = q0(5) + input(5)*(pi/180);
            q(6) = q0(6) + input(6)*(pi/180);

            if self.CheckCollisions(q)
                display('WARNING, COLLISION DETECTED FROM JOINT MANIPULATION.')
                return
            end

            for j = 1:self.model.n                                                             % Loop through joints 1 to 6
                if q(j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                    display(['Reaching joint ' num2str(j) ' limit'])
                    return
                elseif q(j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                    display(['Reaching joint ' num2str(j) ' limit'])
                    return
                end
            end
            
            self.model.animate(q);
            self.PlotCamera();
            self.PlotTarget();
            self.ROSSendGoal(q);

        end

        function BotSet(self, ~, input)
            % BotSet - Function called during event where GUI cartesian
            % movment buttons are pressed

            vLambda = 0.01; % Scaling cartesian movment
            wLambda = pi/180; % Scaling angular movement
            
            % Get new translation from button press
            x = input(1)*vLambda;
            y = input(2)*vLambda;
            z = input(3)*vLambda;
            roll = input(4)*wLambda;
            pitch = input(5)*wLambda;
            yaw = input(6)*wLambda;

            % Determine next pose
            Txyz = transl(x,y,z);
            Tr = trotx(roll)*troty(pitch)*trotz(yaw);
            
            T = Txyz*self.model.fkine(self.model.getpos())*Tr;

            q = self.model.ikcon(T, self.model.getpos());

            if self.CheckCollisions(q)
                display('WARNING, COLLISION DETECTED FROM JOINT MANIPULATION.')
                return
            end
            
            for j = 1:self.model.n                                                             % Loop through joints 1 to 6
                if q(j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                    display(['Reaching joint ' num2str(j) ' limit'])
                    return
                elseif q(j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                    display(['Reaching joint ' num2str(j) ' limit'])
                    return
                end
            end

            if self.CheckSingularity(q)
                display('ROBOT APPROACHING SINGULARITY, STOPPING MOVEMENT.')
                return
            end

            self.model.animate(q);
            self.PlotCamera();
            self.PlotTarget();
            self.ROSSendGoal(q);

        end

        function RMRCResults(self, qMatrix, qdot, positionError, angleError, m, epsilon)
            % RMRCResults - Displays robot joints and manipulability for analysis

            for i = 1:6
                figure(2)
                subplot(3,2,i)
                plot(qMatrix(:,i),'k','LineWidth',1)
                title(['Joint ', num2str(i)])
                ylabel('Angle (rad)')
                refline(0,self.model.qlim(i,1));
                refline(0,self.model.qlim(i,2));
                
                figure(3)
                subplot(3,2,i)
                plot(qdot(:,i),'k','LineWidth',1)
                title(['Joint ',num2str(i)]);
                ylabel('Velocity (rad/s)')
                refline(0,0)
            end
            
            figure(4)
            subplot(2,1,1)
            plot(positionError'*1000,'LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Position Error (mm)')
            legend('X-Axis','Y-Axis','Z-Axis')
            
            subplot(2,1,2)
            plot(angleError','LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Angle Error (rad)')
            legend('Roll','Pitch','Yaw')
            figure(5)
            plot(m,'k','LineWidth',1)
            refline(0,epsilon)
            title('Manipulability')

        end

        function CollisionDetected = CheckCollisions (self, q)
            % CheckCollisions - Takes the model and the next joint
            % positions q, constructs ellipsoid points centered around the joint
            % midpoints and checks intersections between ellipsoids and the
            % floor/table
            CollisionDetected = 0;          % 
            
            return
        end

        function NearSingularityM = CheckSingularity(self,q)
            % NearSingularityM - Checks if the robot is close to a
            % singularity, and returns a boolean
            J = self.model.jacob0(q);
            m = sqrt(det(J*J'));
            epsilon = 0.001;  
            if(m < epsilon)  % Check if manipulability is below a certain threshold
                NearSingularityM = 1;
            else
                NearSingularityM = 0;
            end
        end

        function ValidateModel(self)
            % ValidateModel - Used to ensure model is using correct D&H parameters
            load ur3_q.mat q

            for step = 1:10:size(q,1)
                self.model.animate(q(step,:))
                pause(0.01);
            end
        end


    end
end