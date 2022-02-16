% helperUpdateCameraTrajectories update camera trajectories for VisualOdometryExample

% Copyright 2016-2019 The MathWorks, Inc. 
function UpdateCameraTrajectories2(viewId, trajectoryEstimated, posesEstimated)

% Plot the estimated trajectory.
locations = vertcat(posesEstimated.AbsolutePose.Translation);
set(trajectoryEstimated, 'XData', locations(:,1), 'YData', ...
    locations(:,2), 'ZData', locations(:,3));