%% Occupancy grid from png von mathworks
clc;
close all;
clear all;
% Import Image
%filepath = fullfile('\\R311-IMAC25\WS2017_PrjNav_Data\3_Gruppe\HM_Karlstrasse_F8100_OG3_mod.png');
filepath = fullfile('C:\Users\Sysadmin\Documents\Partikelfilter Navi App\Partikelfilter/GPS_all_filled.png');
image = imread(filepath);

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 245;

% Use black and white image as matrix input for binary occupancy grid
occ_grid = robotics.OccupancyGrid(bwimage,4);
bin_occ_grid = robotics.BinaryOccupancyGrid(bwimage,4)

% Simulate robot poses
start = [207,138];
fin = [236,203];
robotPoses = data_simu(start,fin);
robotPoses(:,3) = pi/2;

% plot
figure(1)
show(occ_grid)
maxrange = 6;
angles = [pi/16,pi/8,pi/4,pi/3,-pi/3,-pi/4,-pi/8,-pi/16];
%robotPose = [207,138,pi/2];
for k = 1 : length(robotPoses)
    intsectionPts = rayIntersection(occ_grid,robotPoses(k,:),angles,maxrange,0.7)
    %Plot
    hold on
    intsections = plot(intsectionPts(:,1),intsectionPts(:,2) , '*r'); % Intersection points
    robots = plot(robotPoses(k,1),robotPoses(k,2),'ob'); % Robot pose
    
    for i = 1:length(intsectionPts)
        if isnan(intsectionPts(i,1))
            nonrays = plot([robotPoses(k,1),robotPoses(k,1)-maxrange*sin(angles(i))],...
                [robotPoses(k,2),robotPoses(k,2)+maxrange*cos(angles(i))],':*b'); % No intersection ray
        else
            rays = plot([robotPoses(k,1),intsectionPts(i,1)],...
                [robotPoses(k,2),intsectionPts(i,2)],'-b'); % Plot intersecting rays
        end        
    end
    
    legend('Collision Points','Robot Position','No Ray','Rays','Location','SouthEast')
 
end

%% Save all walls(black points) to as mat file walls.mat
figure(2)
show(bin_occ_grid,'grid')
% index = 1;
% tic
% for r = 1 : bin_occ_grid.GridSize(1,1)
%     for c = 1 : bin_occ_grid.GridSize(1,2)
%         if (getOccupancy(bin_occ_grid,[r,c],'grid')) == 1
%             walls(index,:) = [r,c];
%             index = index + 1;
%         end
%     end
% end
% toc
% save('walls_filled', 'walls');

%% Pruefen ob Punkte valide auf der Karte sind
% Testpunkte
% point = [474,998]; % out
% point = [534,789]; % in
% point = [300,1207]; % in
points = robotPoses(:,1:2);
for i = 1 : length(points)
    points(i,1:2) = world2grid(bin_occ_grid,robotPoses(i,1:2));
    in = inpolygon(points(i,1), points(i,2), walls(:,1), walls(:,2))
    if (getOccupancy(bin_occ_grid,point,'grid')) == 1
        in = false;
    end
end

