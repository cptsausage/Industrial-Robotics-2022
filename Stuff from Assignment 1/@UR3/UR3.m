classdef UR3 < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
        model;

        workspace = [-3 3 -3 3 -0.3 2];  

    end

    methods
        function self = UR3()
            
            self.GetUR3Robot();

            self.PlotUR3();

        end

        function GetUR3Robot(self)
            
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

            self.model = SerialLink(L,'name','UR3');

        end

        function PlotUR3(self)

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

        % Used to ensure model is using correct D&H parameters
        function ValidateModel(self)
            load ur3_q.mat q

            for step = 1:10:size(q,1)
                self.model.animate(q(step,:))
                pause(0.01);
            end
        end
    end
end