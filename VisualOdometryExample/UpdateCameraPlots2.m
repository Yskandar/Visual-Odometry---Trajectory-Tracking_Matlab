% helperUpdateCameraPlots update the camera plots for VisualOdometryExample

% Copyright 2016-2019 The MathWorks, Inc. 
function UpdateCameraPlots2(viewId, camEstimated, posesEstimated)

% Move the estimated camera in the plot.
camEstimated.AbsolutePose = posesEstimated.AbsolutePose(viewId);



