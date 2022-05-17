classdef Workspace
    %WORKSPACE Workspace initialisation
    %   Initialising a workspace
    
    properties
        estopActive
        laserBot
        targetBot
    end
    
    methods
        function obj = Workspace()
            %WORKSPACE Construct an instance of this class
            %   CONSTRUCTOR
            %   Load in all the models
            close all;
            set(0,'DefaultFigureWindowStyle','docked')
            
            obj.estopActive = false;

            hold("on");
            
            % Load in the environment floors
            surf([-1.25,-1.25;1.25,1.25],[-1,1;-1,1],[0.01,0.01;0.01,0.01], ...
               'CData',imread('wood.jpg'),'FaceColor','texturemap');
            surf([-2,-2;2,2],[-3,1;-3,1],[-0.65,-0.65;-0.65,-0.65], ...
               'CData',imread('marble.jpg'),'FaceColor','texturemap');

            % Load in the environment walls
            surf([2,-2;2,-2],[-3,-3;-3,-3],[2.15,2.15;-0.65,-0.65], ...
               'CData',imread('brick-wall.jpg'),'FaceColor','texturemap');
            surf([1,-1;1,-1],[-2.9,-2.9;-2.9,-2.9],[1,1;0,0], ...
               'CData',imread('TargetsImage.jpg'),'FaceColor','texturemap');
            
            % Load in the table model
            [f,v,data] = plyread('table.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3)-0.04 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            % Load in the safety features
            [f,v,data] = plyread('e-stop.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1), v(:,2)-0.9, v(:,3)+0.3 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            % Load in the robots
            obj.laserBot = LaserBot();
            hold("on");
            obj.targetBot = TargetBot();
            

        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.estopActive + inputArg;
        end
    end
end

