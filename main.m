%% Occupancy grid from png von mathworks
clc;
close all;
clear all;
tic
% Import Image
%filepath = fullfile('\\R311-IMAC25\WS2017_PrjNav_Data\3_Gruppe\HM_Karlstrasse_F8100_OG3_mod.png');
% filepath = fullfile('C:\Users\Sysadmin\Documents\Partikelfilter Navi App\Partikelfilter/GPS_all_filled.png');
filepath = fullfile('D:\Partikelfilter\Partikelfilter/GPS_all_filled.png');
% Jussi stick
filepath = fullfile('/Volumes/NO NAME/7.Semester/Partikelfilter20181023/Partikelfilter/GPS_all_black.png');
% filepath = fullfile('D:\Partikelfilter\Partikelfilter/GPS_all_black.png');
load('walls_black')
image = imread(filepath);

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 245;

% Use black and white image as matrix input for binary occupancy grid
occ_grid = robotics.OccupancyGrid(bwimage,4);
bin_occ_grid = robotics.BinaryOccupancyGrid(bwimage,4);

% Simulate robot poses
start = [207,138];
fin = [236,203];
robotPoses = data_simu(start,fin);
robotPoses(:,3) = pi/2;

% plot
figure(1)
show(occ_grid);
q_range = randn(1) * 0.8;
q_angle = randn(1) * 0.5;
maxrange = 6 + q_range;
angles = [pi/16,pi/8,pi/4,pi/3,-pi/3,-pi/4,-pi/8,-pi/16];
angles = angles + q_angle;
%robotPose = [207,138,pi/2];
for k = 1 : length(robotPoses)
    intsectionPts = rayIntersection(occ_grid,robotPoses(k,:),angles,maxrange,0.7);
    %Plot
    hold on
    intsections = plot(intsectionPts(:,1),intsectionPts(:,2) , '*g'); % Intersection points
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
    grid on;
    legend('Collision Points','Robot Position','No Ray','Rays','Location','SouthEast')
    hold on
end

%% Save all walls(black points) to as mat file walls.mat
% figure(2)
% show(bin_occ_grid,'grid')
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
% save('walls_black', 'walls');

%% Init Partikel
% load('walls_black.mat')
tic
N = 10000;
particles = zeros(N,4);
particles(:,4) = 1/N;
particles_resampled = zeros(N,4);
particles_resampled(:,4) = 1/N;
for i = 1 : N
    particles(i,1:2) = gen_random_particle([175,165], [24,200]);
    out_of_map = checkOccupancy(occ_grid, particles(i,1:2));
    while out_of_map
        particles(i,1:2) = gen_random_particle([175,165], [24,200]);
        out_of_map = checkOccupancy(occ_grid, particles(i,1:2));
    end
    particles(i,3) = rand(1) * pi/18;
end
% Pruefen ob Punkte valide auf der Karte sind
% index = 1;
% for i = 1 : length(particles)
    %particles(i,1:2) = world2grid(bin_occ_grid,particles(i,1:2));
%     walls_world = grid2world(bin_occ_grid, walls(:,1:2));
%     in = inpolygon(particles(i,1), particles(i,2), walls_world(:,1), walls_world(:,2));
%     if (checkOccupancy(occ_grid,particles(i,1:2))) == 0
%        valid_particles(i,:) = grid2world(bin_occ_grid, particles(i,1:2));
%        valid_particles(index,:) = particles(i,:);
%        index = index + 1;
%     end
% end
%save('valid_particles', 'valid_particles')
sum_part = length(particles);
% part_plot = plot(valid_particles(:,1), valid_particles(:,2), '.r');
% hold on
plot(particles(:,1), particles(:,2), '.r')
toc
%% Pruefen ob Punkte valide auf der Karte sind
% Testpunkte
% point = [474,998]; % out
% point = [534,789]; % in
% point = [300,1207]; % in
% points = robotPoses(:,1:2);
% for i = 1 : length(points)
%     points(i,1:2) = world2grid(bin_occ_grid,robotPoses(i,1:2));
%     in = inpolygon(points(i,1), points(i,2), walls(:,1), walls(:,2));
%     if (getOccupancy(bin_occ_grid,points(i,:),'grid')) == 1
%         in = false;
%     end
% end









