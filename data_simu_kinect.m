function [intsectionPts,plots] = data_simu_kinect(occ_grid,robotPose,fov,maxrange)
figure(1)
legend off 
plots.rays = plot(NaN, '-g');
plots.nonrays = plot(NaN, ':g');

%intersectionPts = rayIntersection(map,pose,angles,maxrange,threshold)
angle_step = deg2rad(60/10);
fov_plot(1) = robotPose(1,3) - deg2rad(30);
for d = 2 : 10
    fov_plot(d) = fov_plot(d-1) + angle_step;
    if fov_plot(d) > pi
        diff = fov_plot(d) - pi;
        fov_plot(d) = -(pi-diff);
    end
end

intsectionPts = rayIntersection(occ_grid,robotPose(1,:),fov,maxrange,0.7);

plots.intsections = plot(intsectionPts(:,1),intsectionPts(:,2), '.y'); % Intersection points
plots.robot       = plot(robotPose(1,1), robotPose(1,2), '*m'); % Robot pose

for i = 1:length(intsectionPts)
    if isnan(intsectionPts(i,1))
        plots.nonrays = plot([robotPose(1,1), robotPose(1,1) + maxrange * cos(fov_plot(i))],...
                       [robotPose(1,2), robotPose(1,2) + maxrange * sin(fov_plot(i))], ':g'); % No intersection ray
        intsectionPts(i,3) = 0;
    else
        plots.rays = plot([robotPose(1,1), intsectionPts(i,1)],...
                    [robotPose(1,2), intsectionPts(i,2)], '-g'); % Plot intersecting rays
        intsectionPts(i,3) = sqrt( (intsectionPts(i,1) - robotPose(1,1) )^2 ...
                                 + (intsectionPts(i,2) - robotPose(1,2) )^2 );
    end
end

end

