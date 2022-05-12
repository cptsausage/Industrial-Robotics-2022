clc
clear
UR3Testy = UR5();

centerPoint1 = [0,0,0];
radii1 = [1,1,1];
centrePoints = zeros(1,3,6);
radiis = zeros(1,3,6)
centrePoint(:,:,1) = [0,0,0]
centrePoint(:,:,2) = [0,0.1,0]
centrePoint(:,:,3) = [0,0,0]
centrePoint(:,:,4) = [0,0,0]
centrePoint(:,:,5) = [0,0,0]
centrePoint(:,:,6) = [0,0,0]
radiis(:,:,1) = [0.1,0.1,0.1]
radiis(:,:,2) = [0.1,0.1,0.1]
radiis(:,:,3) = [0.1,0.1,0.1]
radiis(:,:,4) = [0.1,0.1,0.1]
radiis(:,:,5) = [0.1,0.1,0.1]
radiis(:,:,6) = [0.1,0.1,0.1]
X = zeros(21,21,6)
Y = zeros(21,21,6)
Z = zeros(21,21,6)

for i = 1:6
    [X(:,:,i),Y(:,:,i),Z(:,:,i)] = ellipsoid( centrePoint(:,1,i), centrePoint(:,2,i), centrePoint(:,3,i), radiis(:,1,i), radiis(:,2,i), radiis(:,3,i) );
end

%[X2,Y2,Z2] = ellipsoid( centerPoint1(1), centerPoint1(2), centerPoint1(3), radii1(1), radii1(2), radii1(3) );
for i = 1:6
    Xtemp = X(:,:,i)
    Ytemp = Y(:,:,i)
    Ztemp = Z(:,:,i)
    Xtemp2 = Xtemp(:)
    Ytemp2 = Ytemp(:)
    Ztemp2 = Ztemp(:)
    UR3Testy.model.points{i} = [Xtemp2,Ytemp2,Ztemp2]
    warning off;
    UR3Testy.model.faces{i} = delaunay(UR3Testy.model.points{i});    
    warning on;
end

 [X,Y,Z] = ellipsoid( centerPoint1(1), centerPoint1(2), centerPoint1(3), radii1(1), radii1(2), radii1(3) );
for i = 1:6
    UR3Testy.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    UR3Testy.model.faces{i} = delaunay(UR3Testy.model.points{i});    
    warning on;
end

UR3Testy.model.plot3d([0,0,0,0,0,0]);
axis equal
camlight
UR3Testy.model.teach