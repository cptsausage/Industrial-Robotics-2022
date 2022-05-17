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
            
            obj.estopActive = false;

            hold("on");
            
            % Load in the environment floors
            surf([-1.25,-1.25;1.25,1.25],[-1,1;-1,1],[0.01,0.01;0.01,0.01], ...
               'CData',imread('wood.jpg'),'FaceColor','texturemap');
            surf([-2,-2;2,2],[-2,2;-2,2],[-0.75,-0.75;-0.75,-0.75], ...
               'CData',imread('marble.jpg'),'FaceColor','texturemap');

            % Load in the environment walls
            surf([2,-2;2,-2],[-2,-2;-2,-2],[2,2;-0.75,-0.75], ...
               'CData',imread('brick-wall.jpg'),'FaceColor','texturemap');
            surf([1,-1;1,-1],[-1.9,-1.9;-1.9,-1.9],[1.75,1.75;0.75,0.75], ...
               'CData',imread('laser_warning.jpg'),'FaceColor','texturemap');
            
            % Load in the table model
            [f,v,data] = plyread('new_table_3.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3)-0.04 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            % Load in the safety features
            [f,v,data] = plyread('estop.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1), v(:,2)-0.8, v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            [f,v,data] = plyread('dennis.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1), v(:,2)-1.3, v(:,3)-0.7 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            hold("on");
            

        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.estopActive + inputArg;
        end
    end
end

