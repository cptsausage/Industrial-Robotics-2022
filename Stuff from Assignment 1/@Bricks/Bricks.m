classdef Bricks
    % UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
        brickLength = 0.25;
        brickHeight = 0.05;
        brick;
        brickUsed;
        brickCount;
        workspace = [-2 2 -2 2 -0.3 2];

    end

    methods
        function self = Bricks(poses)
            self.brickCount = length(poses);
            self.brickUsed = zeros(1,self.brickCount);
            
            for i = 1:self.brickCount
                name = ['Brick',num2str(i)];
                [faceData,vertexData] = plyread('Brick.ply','tri');
                L1 = Link('alpha',-pi/2,'a',0,'d',0.3,'offset',0);
                self.brick{i} = SerialLink(L1,'name',name);
                self.brick{i}.faces = {faceData,[]};
                self.brick{i}.points = {vertexData, []};
                self.brick{i}.base = poses(:,:,i);
            end 

            self.PlotBricks();
            
        end

        function PlotBricks(self)
            for i = 1:self.brickCount
                    plot3d(self.brick{i},0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            end
        end
    end
end