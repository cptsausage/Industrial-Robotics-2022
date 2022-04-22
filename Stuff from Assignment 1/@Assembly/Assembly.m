classdef Assembly < handle
    %Assembly Assembly of a wall using bricks and robots
    %   This class uses the Bricks, UR3 and LinearUR5 classes to plan and
    %   assembly a brick wall

    properties
        % Input base transforms for robots here
        ur3Base = transl(0,0.3,0)*trotz(-pi/2);
        ur5Base = transl(0.4,-0.6,0)* trotx(pi/2) * troty(pi/2);

        % Properties for robot arm objects
        ur3;
        ur5;

        % Used in placing random bricks within arms reach, using
        % pre-calculated values for quick testing
        ur3Reach = 0.56423;
        ur5Reach = 1.1541;

        % Default positions/waypoints for bots to minimise collision
        ur3HoldingPosition = deg2rad([0,-90,45,-45,-90,0]);
        ur5HoldingPosition = [-0.4,0,0,deg2rad(45),deg2rad(-45),deg2rad(-90),0];

        % Variable holding robot's target brick in the array
        ur3Target;
        ur5Target;

        % Matices of robot arm joint steps for output
        ur3Steps;
        ur5Steps;

        % Steps used in calculating qMatrix
        steps = 50;

        % Time between frames in model animate loop
        pausetime = 0.01;

        % An array of brick objects
        bricks; 
        
        % An array of poses for brick positions to construct a wall
        wallPlan;
        
        % An array of boolean values to flag if spots in the wall have been
        % filled
        spotVacant;

        % Workspace area used for plotting
        workspace = [-3 3 -3 3 -0.3 2];

    end

    methods

        function self = Assembly()

            % Plot floor and backdrop of workspace
            surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            hold on
            surf([-3,3;-3,3],[3,3;3,3],[2,2;0.01,0.01],'CData',imread('skyline1.jpg'),'FaceColor','texturemap');
            surf([3,3;3,3],[-3,3;-3,3],[2,2;0.01,0.01],'CData',imread('skyline1.jpg'),'FaceColor','texturemap');
            
            self.PlotBots();
            display('Bots are ready...')

        end

        function PlotBots(self)    
            % Plot robots in workspace
            self.ur3 = UR3();
            self.ur5 = LinearUR5(false);

            % Move robots to specified poses
            self.ur3.model.base = self.ur3Base;
            self.ur3.model.animate(self.ur3HoldingPosition);

            % Initiate a variable to log joint trajectory over wall
            % assembly
            self.ur3Steps(1,:) = self.ur3.model.getpos();

            self.ur5.model.base = self.ur5Base;
            self.ur5.model.animate(self.ur5HoldingPosition);

            % Initiate a variable to log joint trajectory over wall
            % assembly
            self.ur5Steps(1,:) = self.ur5.model.getpos();

        end

        function SafetyPlot(self) 
            %% Plot X-Y reach of ur3
            qlim = self.ur3.model.qlim;
            stepRads = deg2rad(1);
            pointCloudeSize = prod(floor((qlim(1,2)-qlim(1,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                q = [q1,0,0,deg2rad(-90),deg2rad(-90),0];
                tr = self.ur3.model.fkine(q);
                pointCloud(counter,:) = tr(1:3,4)';
                counter = counter + 1;
            end
            XYur3 = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

            %% Determine maximum reach of ur3
            self.ur3Reach = sqrt(sum(((self.ur3.model.base(1:3,4))'-pointCloud(1,:)).^2));

            %% Plot Y-Z reach of ur3
            pointCloudeSize = 2 * prod(floor((qlim(2,2)-qlim(2,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    q = [0,q2,0,deg2rad(-90),deg2rad(-90),0];
                    tr = self.ur3.model.fkine(q);
                    pointCloud(counter,:) = tr(1:3,4)';
                    counter = counter + 1;
                    q = [deg2rad(180),q2,0,deg2rad(-90),deg2rad(-90),0];
                    tr = self.ur3.model.fkine(q);
                    pointCloud(counter,:) = tr(1:3,4)';
                    counter = counter + 1;
                end
            YZur3 = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

            disp(['Maximum reach of UR3 arm: ' num2str(self.ur3Reach) ' m'])
            input('Press Enter to continue...')

            %% Plot X-Y reach of ur5
            qlim = self.ur5.model.qlim;
            pointCloudeSize = prod(floor((qlim(1,2)-qlim(1,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            for q1 = qlim(1,1):qlim(1,1)/-10:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    q = [q1,q2,deg2rad(-90),0,0,deg2rad(-90),0];
                    tr = self.ur5.model.fkine(q);
                    pointCloud(counter,:) = tr(1:3,4)';
                    counter = counter + 1;
                end
            end
            XYur5 = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'g.');

            %% Determine maximum reach of ur5
            self.ur5Reach = sqrt(sum(((self.ur5.model.base(1:3,4))'-pointCloud(1,:)).^2));
            
            %% Plot Y-Z reach of ur5
            pointCloudeSize = prod(floor((qlim(1,2)-qlim(1,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            for q1 = qlim(1,1):qlim(1,1)/-10:qlim(1,2)
                for q3 = qlim(3,1):stepRads:qlim(3,2)
                    q = [q1,0,q3,0,0,deg2rad(-90),0];
                    tr = self.ur5.model.fkine(q);
                    pointCloud(counter,:) = tr(1:3,4)';
                    counter = counter + 1;
                end
            end
            YZur5 = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'g.');

            disp(['Maximum reach of UR5 arm: ' num2str(self.ur5Reach) ' m'])
            input('Press Enter to continue...')

            % Make existing plots invisible for clarity
            set(XYur3,'Visible','off');
            set(YZur3,'Visible','off');
            set(XYur5,'Visible','off');
            set(YZur5,'Visible','off');

            
            %% Plot Volume of maximum reach for UR3
            stepRads = deg2rad(30);
            qlim = self.ur3.model.qlim;
            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic
            
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                % Don't need to worry about joint 6, just assume it=0
                                q6 = 0;
            %                     for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q = [q1,q2,q3,q4,q5,q6];
                                    tr = self.ur3.model.fkine(q);                        
                                    pointCloud(counter,:) = tr(1:3,4)';
                                    counter = counter + 1; 
                                    if mod(counter/pointCloudeSize * 100,1) == 0
                                        display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                    end
            %                     end
                            end
                        end
                    end
                end
            end
             
            VolumeUR3 = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

            % Calculate approx. volume (hemisphere)
            Volume = (2/3)*pi*(self.ur3Reach)^3;
            display(['Approximate volume of UR3 arm: ' num2str(Volume) ' m^3'])
            input('Press Enter to continue...')
            set(VolumeUR3,'Visible','off');

            
            %% Plot Volume of maximum reach for LinearUR5
%             stepRads = deg2rad(45);
%             qlim = self.ur5.model.qlim;
%             % Don't need to worry about joint 6
%             pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1));
%             pointCloud = zeros(pointCloudeSize,3);
%             counter = 1;
%             tic
%             
%             for q1 = qlim(1,1):qlim(1,1)/-2:qlim(1,2)
% %                 q1 = 0;
%                 for q2 = qlim(2,1):stepRads:qlim(2,2)
%                     for q3 = qlim(3,1):stepRads:qlim(3,2)
%                         for q4 = qlim(4,1):stepRads:qlim(4,2)
%                             for q5 = qlim(5,1):stepRads:qlim(5,2)
%                                 for q6 = qlim(6,1):stepRads:qlim(6,2)
%                                     %ignore joint 7
%                                     q = [q1,q2,q3,q4,q5,q6,0];
%                                     tr = self.ur5.model.fkine(q);                        
%                                     pointCloud(counter,:) = tr(1:3,4)';
%                                     counter = counter + 1; 
%                                     if mod(counter/pointCloudeSize * 100,1) == 0
%                                         display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
%                                     end
%                                 end
%                             end
%                         end
%                     end
%                 end
%             end
%             
%             VolumeUR5 = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
% 
%             % Calculate approx. volume (hemisphere + half-cylinder)
%             h = abs(self.ur5.model.qlim(1,1) - self.ur5.model.qlim(1,2));
%             Volume = (2/3)*pi*(self.ur5Reach)^3 + (1/2)*pi*((self.ur5Reach)^2)*h;
%             display(['Approximate volume of UR5 arm: ' num2str(Volume) ' m^3'])
%             input('Press Enter to continue...')
%             set(VolumeUR3,'Visible','off');
%             input('Press Enter to continue...')
%             set(VolumeUR5,'Visible','off');

            %% Plot safety cones and tape around arm workspace

            % Find midpoint between two arms
            % Account for UR5 'base' offset from linear prismatic joint, may need to change according to setup
            midy = (self.ur3.model.base(2,4) + self.ur5.model.base(2,4))/2;
            midx = (self.ur3.model.base(1,4) + self.ur5.model.base(1,4))/2 + self.ur5.model.qlim(1,1)/2; 
            
            
            [f,v,data] = plyread('Cone.ply', 'tri');

            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            tapeIndex = 1;

            point = zeros(4,3);

            for offsetx = [midx-1.8*self.ur5Reach midx+1.8*self.ur5Reach]
                for offsety = [midy-1.8*self.ur5Reach midy+1.8*self.ur5Reach]
                    trisurf(f,v(:,1)+offsetx, v(:,2)+offsety, v(:,3) ...
                        ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp','EdgeLighting','flat');
                    point(tapeIndex,:) = [offsetx offsety 0.05];
                    tapeIndex = tapeIndex + 1;
                end
            end

            plot3([point(1,1),point(2,1)],[point(1,2),point(2,2)],[point(1,3),point(2,3)], '-oy');
            plot3([point(2,1),point(4,1)],[point(2,2),point(4,2)],[point(2,3),point(4,3)], '-oy');
            plot3([point(4,1),point(3,1)],[point(4,2),point(3,2)],[point(4,3),point(3,3)], '-oy');
            plot3([point(3,1),point(1,1)],[point(3,2),point(1,2)],[point(3,3),point(1,3)], '-oy');

            display('Finished advising safety barricades based on robot arm reach...')
            


        end

        function RandomBricks(self,TotalBricks)
            % Used for testing purposes, setting location of bricks randomly
            poses = zeros(4,4,TotalBricks);
            for i = 1:1:TotalBricks

                % Divide the bricks in reach between the two robots
                if rem(i, 2) == 1
                    theta = deg2rad(randi(360,1,1)); % Generate random angle for placement
                    h = 0.5 + rand(1)*(self.ur3Reach-0.5); % Generate random distance from base
                    x = self.ur3.model.base(1,4) + h*cos(theta);
                    y = self.ur3.model.base(2,4) + h*sin(theta);
                    poses(:,:,i) = transl(x,y,0);
                else
                    theta = deg2rad(randi(360,1,1)); % Generate random angle for placement
                    h = 0.5 + rand(1)*(self.ur5Reach-0.5); % Generate random distance from base
                    x = self.ur5.model.base(1,4) + h*cos(theta);
                    y = self.ur5.model.base(2,4) + h*sin(theta);
                    poses(:,:,i) = transl(x,y,0);
                end
            end
            self.bricks = Bricks(poses);
        end

        function SpecificBricks(self,BrickPoses)
            
            % Used to place a set of bricks in specific location
            poses = zeros(4,4,length(BrickPoses));
            for i = 1:length(BrickPoses)
                x = BrickPoses(i,1);
                y = BrickPoses(i,2);
                z = BrickPoses(i,3);
                poses(:,:,i) = transl(x,y,z);
            end

            self.bricks = Bricks(poses);

        end

        function Construction(self)
            % A function that begins by planning the end positions of
            % bricks in a wall, and looping through robot joint trajectory
            % calculation and animation until each brick is in a position

            self.PlanWall(); % Plan brick positions in wall
            display('Final brick poses calculated, initiating wall assembly...')

            while self.spotVacant(length(self.spotVacant)) == 0
                self.LocateBricks(); % Locate next brick to grab for each arm
                self.PickupBricks(); % Move arm joints through trajectories to grab bricks
                self.TakeBricks(); % Pull brick to holding waypoint and avoid collision
                self.PlaceBricks(); % Place brick in vacant spot
            end

            % Notify when wall is completed and save logged joint trajectories of
            % robots throughout assembly as a file for use later
            display('Wall completed! Saving joint steps...')
            qur3 = self.ur3Steps;
            qur5 = self.ur5Steps;
            save('qur3.mat', 'qur3')
            save('qur5.mat', 'qur5')

        end

        function PlanWall(self)
            % Plan a 3x3 wall between arms, 
            % given brick dimensions in Bricks class
            self.spotVacant = zeros(length(self.bricks.brick));
            self.wallPlan = zeros(4,4,length(self.bricks.brick));
            % Account for UR5 'base' offset from linear prismatic joint, may need to change according to setup
            wally = (self.ur3.model.base(2,4) + self.ur5.model.base(2,4))/2;
            wallx = (self.ur3.model.base(1,4) + self.ur5.model.base(1,4))/2 + self.ur5.model.qlim(1,1)/2; 
            wallz = self.ur3.model.base(3,4);
            count = 1;
            for z = wallz:self.bricks.brickHeight:wallz + self.bricks.brickHeight*2
                % Will need to change whether built along x or y plane when
                % given base poses and facing
                for x = wallx:self.bricks.brickLength:wallx + self.bricks.brickLength*2
                    self.wallPlan(:,:,count) = transl(x,wally,z);
                    count = count + 1;
                end
%                 for y = wally:self.bricks.brickLength:wally + self.bricks.brickLength*2
%                     self.wallPlan(:,:,count) = transl(wallx,y,z);
%                 end
            end
        end
        
        function LocateBricks(self)
            % Find closest brick to UR3 that has not yet been used
            closest = self.ur3Reach;
            self.ur3Target = 0; % If Target remains 0, there are no bricks within reach
            for i = 1:length(self.bricks.brick)
                distance = sqrt(sum(((self.ur3.model.base(1:3,4))'-self.bricks.brick{i}.base(1:3,4)').^2));
                if distance < closest && self.bricks.brickUsed(i) == 0
                    self.ur3Target = i;
                    closest = distance;
                end
            end
            % Set brick status as 'used' so it is not picked up again
            if self.ur3Target ~= 0
                self.bricks.brickUsed(self.ur3Target) = 1;
            end

            % Find closest brick to UR5 that has not yet been used
            closest = self.ur5Reach;
            self.ur5Target = 0; % If Target remains 0, there are no bricks within reach
            for i = 1:length(self.bricks.brick)
                distance = sqrt(sum(((self.ur5.model.base(1:3,4))'-self.bricks.brick{i}.base(1:3,4)').^2));
                if distance < closest && self.bricks.brickUsed(i) == 0
                    self.ur5Target = i;
                    closest = distance;
                end
            end
            % Set brick status as 'used' so it is not picked up again
            if self.ur5Target ~= 0
                self.bricks.brickUsed(self.ur5Target) = 1;
            end
        end
     
        function PickupBricks(self)

            % Get qMatrix for ur3 to reach target brick pose (rot around x so end effector of robot will face down)
            if self.ur3Target ~= 0 % Check that there is a target for the robot
                targetBrickUR3 = self.bricks.brick{self.ur3Target}.base * trotx(pi);
                targetBrickUR3(3,4) = targetBrickUR3(3,4) + 2*self.bricks.brickHeight;
                newUR3Q = self.ur3.model.ikcon(targetBrickUR3,self.ur3.model.getpos());
                qUR3Matrix = jtraj(self.ur3.model.getpos(),newUR3Q,self.steps);
            else
                qUR3Matrix = jtraj(self.ur3.model.getpos(),self.ur3HoldingPosition,self.steps);
            end
            
            % Get qMatrix for ur5 to reach target brick pose (rot around x so end effector of robot will face down)
            if self.ur5Target ~= 0 % Check that there is a target for the robot
                targetBrickUR5 = self.bricks.brick{self.ur5Target}.base * trotx(pi);
                targetBrickUR5(3,4) = targetBrickUR5(3,4) + 2.5*self.bricks.brickHeight;
                newUR5Q = self.ur5.model.ikcon(targetBrickUR5,self.ur5.model.getpos());
                qUR5Matrix = jtraj(self.ur5.model.getpos(),newUR5Q,self.steps);
            else
                qUR5Matrix = jtraj(self.ur5.model.getpos(),self.ur5HoldingPosition,self.steps);
            end

            % Loop through qMatrix for simultaneous movement of arms
            for i = 1:self.steps-self.steps/5
                self.ur3.model.animate(qUR3Matrix(i,:));
                self.ur3Steps(end+1,:) = self.ur3.model.getpos();
                self.ur5.model.animate(qUR5Matrix(i,:));
                self.ur5Steps(end+1,:) = self.ur5.model.getpos();
                pause(self.pausetime);
            end

            % Re-adjust trajectory once near target to minimise error
            if self.ur3Target ~= 0
                qUR3Matrix = jtraj(self.ur3.model.getpos(),newUR3Q,self.steps/5);
            end
            if self.ur5Target ~= 0
                qUR5Matrix = jtraj(self.ur5.model.getpos(),newUR5Q,self.steps/5);
            end

            for i = 1:self.steps/5
                self.ur3.model.animate(qUR3Matrix(i,:));
                self.ur3Steps(end+1,:) = self.ur3.model.getpos();
                self.ur5.model.animate(qUR5Matrix(i,:));
                self.ur5Steps(end+1,:) = self.ur5.model.getpos();
                pause(self.pausetime)
            end

        end

        function TakeBricks(self)
            % Move arms and bricks back to the defined 'holding position'
            % to minimise collisions

            qUR3Matrix = jtraj(self.ur3.model.getpos(),self.ur3HoldingPosition,self.steps);
            qUR5Matrix = jtraj(self.ur5.model.getpos(),self.ur5HoldingPosition,self.steps);

            for i = 1:self.steps
                self.ur3.model.animate(qUR3Matrix(i,:));
                self.ur3Steps(end+1,:) = self.ur3.model.getpos();
                self.ur5.model.animate(qUR5Matrix(i,:));
                self.ur5Steps(end+1,:) = self.ur5.model.getpos();
                if self.ur3Target ~= 0
                    self.bricks.brick{self.ur3Target}.base = (self.ur3.model.fkine(self.ur3.model.getpos()))*trotx(pi);
                    self.bricks.brick{self.ur3Target}.base(3,4) = self.bricks.brick{self.ur3Target}.base(3,4) - self.bricks.brickHeight;
                    animate(self.bricks.brick{self.ur3Target},0);
                end
                if self.ur5Target ~= 0
                    self.bricks.brick{self.ur5Target}.base = (self.ur5.model.fkine(self.ur5.model.getpos()))*trotx(pi);
                    self.bricks.brick{self.ur5Target}.base(3,4) = self.bricks.brick{self.ur5Target}.base(3,4) - 1.5*self.bricks.brickHeight;
                    animate(self.bricks.brick{self.ur5Target},0);
                end
                pause(self.pausetime)
            end

        end

        function PlaceBricks(self)

            if self.ur3Target ~= 0
                spot = 1;
                % Look for the next vacant spot in the wall
                while self.spotVacant(spot) == 1
                    spot = spot + 1;
                end
    
                % Spot is no longer vacant
                self.spotVacant(spot) = 1;

                % Calculate pose and qMatrix for vacant spot
                spotpose = self.wallPlan(:,:,spot)*trotx(pi);
                spotpose(3,4) = spotpose(3,4) + self.bricks.brickHeight;
                newUR3Q = self.ur3.model.ikcon(spotpose,self.ur3.model.getpos());
            else
                newUR3Q = self.ur3.model.getpos();
            end

            qUR3Matrix = jtraj(self.ur3.model.getpos(),newUR3Q,self.steps);

            if self.ur5Target ~= 0
                spot = 1;
                % Look for the next vacant spot in the wall
                while self.spotVacant(spot) == 1
                    spot = spot + 1;
                end

                % Spot is no longer vacant
                self.spotVacant(spot) = 1;
    
                % Calculate pose and qMatrix for vacant spot
                spotpose = self.wallPlan(:,:,spot)*trotx(pi);
                spotpose(3,4) = spotpose(3,4) + 1.5*self.bricks.brickHeight;
                newUR5Q = self.ur5.model.ikcon(spotpose,self.ur5.model.getpos());
            else
                newUR5Q = self.ur5.model.getpos();
            end

            qUR5Matrix = jtraj(self.ur5.model.getpos(),newUR5Q,self.steps);

            for i = 1:self.steps-self.steps/5
                self.ur3.model.animate(qUR3Matrix(i,:));
                self.ur3Steps(end+1,:) = self.ur3.model.getpos();
                self.ur5.model.animate(qUR5Matrix(i,:));
                self.ur5Steps(end+1,:) = self.ur5.model.getpos();
                if self.ur3Target ~= 0
                    self.bricks.brick{self.ur3Target}.base = (self.ur3.model.fkine(self.ur3.model.getpos()))*trotx(pi);
                    self.bricks.brick{self.ur3Target}.base(3,4) = self.bricks.brick{self.ur3Target}.base(3,4) - self.bricks.brickHeight;
                    animate(self.bricks.brick{self.ur3Target},0);
                end
                if self.ur5Target ~= 0
                    self.bricks.brick{self.ur5Target}.base = (self.ur5.model.fkine(self.ur5.model.getpos()))*trotx(pi);
                    self.bricks.brick{self.ur5Target}.base(3,4) = self.bricks.brick{self.ur5Target}.base(3,4) - 1.5*self.bricks.brickHeight;
                    animate(self.bricks.brick{self.ur5Target},0);
                end
                pause(self.pausetime)
            end

            % Re-adjust trajectory once near target to minimise error
            qUR3Matrix = jtraj(self.ur3.model.getpos(),newUR3Q,self.steps/5);
            qUR5Matrix = jtraj(self.ur5.model.getpos(),newUR5Q,self.steps/5);
            
            for i = 1:self.steps/5
                self.ur3.model.animate(qUR3Matrix(i,:));
                self.ur3Steps(end+1,:) = self.ur3.model.getpos();
                self.ur5.model.animate(qUR5Matrix(i,:));
                self.ur5Steps(end+1,:) = self.ur5.model.getpos();
                if self.ur3Target ~= 0
                    self.bricks.brick{self.ur3Target}.base = (self.ur3.model.fkine(self.ur3.model.getpos()))*trotx(pi);
                    self.bricks.brick{self.ur3Target}.base(3,4) = self.bricks.brick{self.ur3Target}.base(3,4) - self.bricks.brickHeight;
                    animate(self.bricks.brick{self.ur3Target},0);
                end
                if self.ur5Target ~= 0
                    self.bricks.brick{self.ur5Target}.base = (self.ur5.model.fkine(self.ur5.model.getpos()))*trotx(pi);
                    self.bricks.brick{self.ur5Target}.base(3,4) = self.bricks.brick{self.ur5Target}.base(3,4) - 1.5*self.bricks.brickHeight;
                    animate(self.bricks.brick{self.ur5Target},0);
                end
                pause(self.pausetime)
            end

            % Return to holding positions
            qUR3Matrix = jtraj(self.ur3.model.getpos(),self.ur3HoldingPosition,self.steps);
            qUR5Matrix = jtraj(self.ur5.model.getpos(),self.ur5HoldingPosition,self.steps);

            for i = 1:self.steps
                self.ur3.model.animate(qUR3Matrix(i,:));
                self.ur3Steps(end+1,:) = self.ur3.model.getpos();
                self.ur5.model.animate(qUR5Matrix(i,:));
                self.ur5Steps(end+1,:) = self.ur5.model.getpos();
                pause(self.pausetime);
            end

        end
    end
end