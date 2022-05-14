clc
clear
UR3Testy = UR5();


%     L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
%     L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
%     L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
%     L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
%     L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
%     L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

    L1 = [0.0892 0 -pi/2 0]
    L2 = [0.1357 0.425 -pi -pi/2]
    L3 = [0.1197 0.39243 pi 0]
    L4 = [0.093 0 -pi/2 -pi/2]
    L5 = [0.093 0 -pi/2 0]
    L6 = [0 0 0 0]
    
    Lall = {L1 L2 L3 L4 L5 L6}
    allradiis = zeros(3,6)
    Lalltemp = [0 0 0 0]
%     for i = 1:6
%         Lalltemp = Lall{i};
%         allradiis(1,i) = -Lalltemp(1)/2 * cos(Lalltemp(3))
%         allradiis(2,i) = -Lalltemp(2)/2 * sin(Lalltemp(4))
%     end

    tr(:,:,1) = UR3Testy.model.base;
    q = [0,0,0,0,0,0]
    L = UR3Testy.model.links;
    distance = zeros(1,6)
    for i = 1 : UR3Testy.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
        
        %%tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
        
        Lalltemp = Lall{i};
        JointMatrix2 = tr(:,:,i)
        
        if i > 1
       
        %JointMatrix = JointMatrix*transl(L(i).d,0,0)
        allradiis(1,i) = JointMatrix2(1,4)/2 + JointMatrix1(1,4)/2
        allradiis(2,i) = JointMatrix2(2,4) %%+ JointMatrix1(2,4)/2
        allradiis(3,i) = JointMatrix2(3,4)/2 +JointMatrix1(3,4)/2
        distance(i) = (JointMatrix2(3,4)/2 - JointMatrix1(3,4)/2)*1.4;
        %%distance(i) = 0.1 
        end
        JointMatrix1 = tr(:,:,i)

    end

    

centerPoint1 = [0,0,0];
radii1 = [1,1,1];
centrePoints = zeros(1,3,6);
radiis = zeros(1,3,6)
centrePoint(:,:,1) = [allradiis(1,1),allradiis(2,1),allradiis(3,1)]
centrePoint(:,:,2) = [allradiis(1,2),allradiis(2,2),allradiis(3,2)]
centrePoint(:,:,3) = [allradiis(1,3),allradiis(2,3),allradiis(3,3)]
centrePoint(:,:,4) = [allradiis(1,4),allradiis(2,4),allradiis(3,4)]
centrePoint(:,:,5) = [allradiis(1,5),allradiis(2,5),allradiis(3,5)]
centrePoint(:,:,6) = [allradiis(1,6),allradiis(2,6),allradiis(3,6)]
radiis(:,:,1) = [0.1,0.1, distance(1)]
radiis(:,:,2) = [0.1,0.1, distance(2)]
radiis(:,:,3) = [0.1,0.1, distance(3)]
radiis(:,:,4) = [0.1,0.1, distance(4)]
radiis(:,:,5) = [0.1,0.1, distance(5)]
radiis(:,:,6) = [0.1,0.1, distance(5)]
X = zeros(21,21,6)
Y = zeros(21,21,6)
Z = zeros(21,21,6)

% for i = 1:6
%     [X(:,:,i),Y(:,:,i),Z(:,:,i)] = ellipsoid( centrePoint(:,1,i), centrePoint(:,2,i), centrePoint(:,3,i), radiis(:,1,i), radiis(:,2,i), radiis(:,3,i) );
% end

% %[X2,Y2,Z2] = ellipsoid( centerPoint1(1), centerPoint1(2), centerPoint1(3), radii1(1), radii1(2), radii1(3) );
% for i =2:3
%     Xtemp = X(:,:,i)
%     Ytemp = Y(:,:,i)
%     Ztemp = Z(:,:,i)
%     Xtemp2 = Xtemp(:)
%     Ytemp2 = Ytemp(:)
%     Ztemp2 = Ztemp(:)
%     ellipsoid(centrePoint(:,1,i), centrePoint(:,2,i),centrePoint(:,3,i),radiis(:,1,i),radiis(:,2,i),radiis(:,3,i))
%     hold on
%     %UR3Testy.model.points{i} = [Xtemp2,Ytemp2,Ztemp2]
%     warning off;
%     %UR3Testy.model.faces{i} = delaunay(UR3Testy.model.points{i});    
%     warning on;
% end



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