%% PROGRAM - PRINTING POINT MATCHING OVER SEVERAL IMAGES (feature extraction method can be changed)
clear;
close all;
clc;

% Loading Images
images = imageDatastore('/poor_light_conditions');
k=1;

figure

for i = 8:1:22 % Going through our images
    subplot(4,4,k) % Creating subplots
    hold on
    I1 = rgb2gray(readimage(images, i));
    I2 = rgb2gray(readimage(images, i+1));
    points1 = detectKAZEFeatures(I1); % Choosing the extraction method
    points2 = detectKAZEFeatures(I2);
    [f1,vpts1] = extractFeatures(I1,points1); % Extracting the features
    [f2,vpts2] = extractFeatures(I2,points2);
    
    indexPairs = matchFeatures(f1,f2) ;
    matchedPoints1 = vpts1(indexPairs(:,1));
    matchedPoints2 = vpts2(indexPairs(:,2));
    showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2); % Printing the matched features
    legend('matched points 1','matched points 2');
    k=k+1
end
