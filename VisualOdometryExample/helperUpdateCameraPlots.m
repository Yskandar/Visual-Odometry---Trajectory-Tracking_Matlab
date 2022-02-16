% helperUpdateCameraPlots update the camera plots for VisualOdometryExample

% Copyright 2016-2019 The MathWorks, Inc. 
function helperUpdateCameraPlots(viewId, camEstimated, camActual, ...
    posesEstimated, posesActual)

% Move the estimated camera in the plot.
camEstimated.AbsolutePose = posesEstimated.AbsolutePose(viewId);

% Move the actual camera in the plot.
camActual.AbsolutePose = rigid3d(posesActual.Orientation{viewId},...
    posesActual.Location{viewId});

