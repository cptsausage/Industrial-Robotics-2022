% Computer Vision Testing
clear;
clf;

%% Load images
refImage = imread('qr-code.png');
% realImage = imread('Book Photo 1.jpg');
% imshowpair(refImage, realImage, 'montage');
title('Reference Image (Left) | Real Image (Right)')

% Find ORBFeatures
refImage_gray = rgb2gray(refImage);
refImage_pts = detectSIFTFeatures(refImage_gray);

% Display strongest features
imshow(refImage);
hold on;
plot(refImage_pts.selectStrongest(50));
hold off;
title('SURF Features in Reference Image');

%% Detect SURF features in real image

cam = webcam('C922 Pro Stream Webcam');

for i = 1:500

realImage = cam.snapshot;
realImage_gray = rgb2gray(realImage);
realImage_pts = detectSIFTFeatures(realImage_gray);
% figure;
% imshow(realImage);
% hold on;
% plot(realImage_pts.selectStrongest(300));
% hold off;
% title('SURF Features in Real Image');

%% Extract and Match Feature Points
[refFeatures, refPoints] = extractFeatures(refImage_gray, refImage_pts);
[realFeatures, realPoints] = extractFeatures(realImage_gray, realImage_pts);

refPairs = matchFeatures(refFeatures, realFeatures);

refPoints_matched = refPoints(refPairs(:,1), :);
realPoints_matched = realPoints(refPairs(:,2),:);
% figure;
% showMatchedFeatures(refImage_gray, realImage_gray, refPoints_matched, realPoints_matched, 'montage');
% title('Matched Points (including outliers)');

%% Locate Object in Scene using Feature Matches
try
[tform, refPoints_inlier, realPoints_inlier] = ...
    estimateGeometricTransform(refPoints_matched, realPoints_matched, "affine");

% figure;
% showMatchedFeatures(refImage_gray, realImage_gray, refPoints_matched, realPoints_matched, 'montage');
% title('Matched Points (inliers only)');

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
catch
    imshow(realImage_gray);
end
end