% helperNormalizeViewSet Translate and scale camera poses to align with ground truth
%  vSet = helperNormalizedViewSet(vSet, groundTruth) returns a view set
%  with the camera poses translated to put the first camera at the origin
%  looking along the Z axes, and scaled to match the scale of the ground
%  truth. vSet is an imageviewset object. groundTruth is a table containing 
%  the actual camera poses.
%
%  See also imageviewset, table

% Copyright 2016-2019 The MathWorks, Inc. 

function vSet = helperNormalizeViewSet(vSet, groundTruth)

camPoses = poses(vSet);

% Move the first camera to the origin.
locations = vertcat(camPoses.AbsolutePose.Translation);
locations = locations - locations(1, :);

locationsGT  = cat(1, groundTruth.Location{1:height(camPoses)});
magnitudes   = sqrt(sum(locations.^2, 2));
magnitudesGT = sqrt(sum(locationsGT.^2, 2));
scaleFactor = median(magnitudesGT(2:end) ./ magnitudes(2:end));

% Rotate the poses so that the first camera points along the Z-axis
R = camPoses.AbsolutePose(1).Rotation';
for i = 1:height(camPoses)
    % Scale the locations
    camPoses.AbsolutePose(i).Translation = camPoses.AbsolutePose(i).Translation * scaleFactor;
    camPoses.AbsolutePose(i).Rotation = camPoses.AbsolutePose(i).Rotation * R;
end

vSet = updateView(vSet, camPoses);
