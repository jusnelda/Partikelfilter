function intsectionPts = data_simu_kinect(occ_grid,robotPose,fov,maxrange)

%maxrange = maxrange + q_range;
%angles = [pi/16,pi/8,pi/4,pi/3,-pi/3,-pi/4,-pi/8,-pi/16];
% angles = -pi/6:0.1047:pi/6;
% angles = [-0.5236,-0.4189,-0.3142,-0.2095,-0.1048,0.1046,0.2093,0.3140,0.4187,0.5234];
%angles = angles + q_angle;
%robotPose = [207,138,pi/2];
%for k = 1 : length(robotPoses)
intsectionPts = rayIntersection(occ_grid,robotPose(1,:),fov,maxrange,0.7);
hold on
intsections = plot(intsectionPts(:,1),intsectionPts(:,2), '.y'); % Intersection points
robots =      plot(robotPose(1,1), robotPose(1,2), '.g'); % Robot pose

for i = 1:length(intsectionPts)
    if isnan(intsectionPts(i,1))
        nonrays = plot([robotPose(1,1), robotPose(1,1) - maxrange * sin(fov(i))],...
                       [robotPose(1,2), robotPose(1,2) + maxrange * cos(fov(i))], ':y'); % No intersection ray
        intsectionPts(i,3) = 0;
    else
        rays = plot([robotPose(1,1), intsectionPts(i,1)],...
                    [robotPose(1,2), intsectionPts(i,2)], '-y'); % Plot intersecting rays
        intsectionPts(i,3) = sqrt( (intsectionPts(i,1) - robotPose(1,1) )^2 ...
                                 + (intsectionPts(i,2) - robotPose(1,2) )^2 );
    end
end

%end
%kinect_data = struct('x', intsectionPts(:,1), 'y', intsectionPts(:,2), 'dist', intsectionPts(:,3));

end

