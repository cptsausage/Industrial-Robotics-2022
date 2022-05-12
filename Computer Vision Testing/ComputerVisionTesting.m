% Computer Vision Testing
clear;
clf;

%% Load images
refImage = imread('qr-code.png');
title('Reference Image (Left) | Real Image (Right)')

% Find SIFTFeatures
refImage_gray = rgb2gray(refImage);
refImage_pts = detectSIFTFeatures(refImage_gray);

% Display strongest features
imshow(refImage);
hold on;
plot(refImage_pts.selectStrongest(50));
hold off;
title('SIFT Features in Reference Image');

%% Detect SIFT features in real image

cam = webcam('C922 Pro Stream Webcam');

for i = 1:500

    realImage = cam.snapshot;
    realImage_gray = rgb2gray(realImage);
    realImage_pts = detectSIFTFeatures(realImage_gray);
    
    %% Extract and Match Feature Points
    [refFeatures, refPoints] = extractFeatures(refImage_gray, refImage_pts);
    [realFeatures, realPoints] = extractFeatures(realImage_gray, realImage_pts);
    
    refPairs = matchFeatures(refFeatures, realFeatures);
    
    refPoints_matched = refPoints(refPairs(:,1), :);
    realPoints_matched = realPoints(refPairs(:,2),:);
    
    %% Locate Object in Scene using Feature Matches
    try
        [tform, refPoints_inlier, realPoints_inlier] = ...
            estimateGeometricTransform(refPoints_matched, realPoints_matched, "affine");
        
        %% Display Location in Real Image
        
        boxPolygon = [1, 1;...
            size(refImage_gray,2), 1;...
            size(refImage_gray,2), size(refImage_gray, 1);...
            1, size(refImage_gray, 1);...
            1, 1];
        
        % Transform polygon onto new image
        
        newBoxPolygon = transformPointsForward(tform, boxPolygon);
        
        imshow(realImage_gray);
        hold on;
        line(newBoxPolygon(:,1),newBoxPolygon(:,2),'Color','y','LineWidth',5);
        hold off;
        title('Object Location in Real Image');
        
        
        uv = newBoxPolygon(1:4,:)';

        principalPoint = [640/2, 480/2];
        scale = 100;

        tu = [principalPoint(1)-scale/2, principalPoint(1)+scale/2];
        tv = [principalPoint(2)-scale/2, principalPoint(2)+scale/2];
        targets = [tu(1) tu(2) tu(2) tu(1) tu(1); tv(1) tv(1) tv(2) tv(2) tv(1)];

        e = [targets(:,1)-uv(:,1);targets(:,2)-uv(:,2);targets(:,3)-uv(:,3);targets(:,4)-uv(:,4)];
    
% Input servoing here
        
        if max(abs(e), [], 'all') < 15
            line(targets(1,:), targets(2,:),'Color','g','LineWidth',5)
            display('Target reached!');
        else
            line(targets(1,:), targets(2,:),'Color','r','LineWidth',5)
            display('Target not reached!');
        end
    catch
        imshow(realImage_gray);
    end
end