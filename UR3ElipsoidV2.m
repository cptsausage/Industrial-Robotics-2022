clc
clear
UR3Testy = UR5(); %%Says UR3 is actually tested on UR5


    L1 = [0.0892 0 -pi/2 0]
    L2 = [0.1357 0.425 -pi -pi/2]
    L3 = [0.1197 0.39243 pi 0]
    L4 = [0.093 0 -pi/2 -pi/2]
    L5 = [0.093 0 -pi/2 0]
    L6 = [0 0 0 0]
    
    %DH Parameters of Links

    Lall = [L1; L2; L3; L4; L5; L6]

    %DH Parameters of 

    tr(:,:,1) = UR3Testy.model.base;
    q = [0,0,0,0,0,0]
    L = UR3Testy.model.links;

    %Unused


centrePoints = zeros(3,6);
radiis = zeros(3,6)

%Setting sizes of matrixes

 for i = 1 : UR3Testy.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);

 end

%Unused, for iterating through location of different links

centrePoint(:,1) = [0,0,0]
centrePoint(:,2) = [0,-Lall(2,2)/2,Lall(2,1)]
centrePoint(:,3) = [Lall(3,2)/2,0,Lall(3,1)]
centrePoint(:,4) = [Lall(4,2),0,Lall(4,1)/2]
centrePoint(:,5) = [Lall(5,2),0,Lall(5,1)/2]
centrePoint(:,6) = [0,0,1/4*Lall(5,1)]

%Setting Centrepoint for 6 different links placing the elipsoid between
%each link, Link 1 is not set due to messing up plot

ThicknessofArm = 0.08
SmallerThicknessofArm = 0.048

%Setting the thickess of each ellipsoid, smaller thickness is used for
%elipsoid with shorter lengths

radiis(:,1) = [0.05,0.05, 0.05]
radiis(:,2) = [ThicknessofArm,Lall(2,2)*0.7, ThicknessofArm]
radiis(:,3) = [Lall(3,2)*0.7,ThicknessofArm, ThicknessofArm]
radiis(:,4) = [SmallerThicknessofArm,SmallerThicknessofArm, Lall(4,1)*1.2]
radiis(:,5) = [SmallerThicknessofArm,SmallerThicknessofArm, Lall(5,1)*1.2]
radiis(:,6) = [SmallerThicknessofArm,SmallerThicknessofArm, 1*Lall(5,1)]

X = zeros(21,21,6)
Y = zeros(21,21,6)
Z = zeros(21,21,6)

%Initialising array of points 

 for i = 1:6
     [X(:,:,i), Y(:,:,i),Z(:,:,i)] = ellipsoid(centrePoint(1,i), centrePoint(2,i), centrePoint(3,i), radiis(1,i),radiis(2,i),radiis(3,i))
     XTemp = X(:,:,i);
     YTemp = Y(:,:,i);
     ZTemp = Z(:,:,i);
     UR3Testy.model.points{i} = [XTemp(:),YTemp(:),ZTemp(:)];
     warning off
     UR3Testy.model.faces{i} = delaunay(UR3Testy.model.points{i});    
     warning on;
 end

%Iterating through faces and applying points

UR3Testy.model.plot3d([0,0,0,0,0,0]);

camlight



UR3Testy.model.teach