%% Occupancy grid from png von mathworks
clc;
close all;
clear all;
tic
% Import Image
%filepath = fullfile('\\R311-IMAC25\WS2017_PrjNav_Data\3_Gruppe\HM_Karlstrasse_F8100_OG3_mod.png');
% filepath = fullfile('C:\Users\Sysadmin\Documents\Partikelfilter Navi App\Partikelfilter/GPS_all_black.png');
%filepath = fullfile('D:\Partikelfilter\Partikelfilter/GPS_all_filled.png');
% Jussi stick
filepath = fullfile('E:\7.Semester\Partikelfilter/Bilder/GPS_all_black.png');
%filepath = fullfile('D:\Partikelfilter\Partikelfilter/Bilder/GPS_all_black.png');
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
q_angle = randn(1) * 0.05;
maxrange = 6 + q_range;
%angles = [pi/16,pi/8,pi/4,pi/3,-pi/3,-pi/4,-pi/8,-pi/16];
% angles = -pi/6:0.1047:pi/6;
angles = [-0.5236,-0.4189,-0.3142,-0.2095,-0.1048,0.1046,0.2093,0.3140,0.4187,0.5234];
%angles = angles + q_angle;
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
N = 5000;
particles = zeros(N,5);
% y bekommt spaeter die Hoehe der Kinect ueber dem Boden
% particles(:,3) = ?
particles(:,5) = 1/N;

for i = 1 : N
    particles(i,1:2) = gen_random_particle([175,165], [24,200]);
    out_of_map = checkOccupancy(occ_grid, particles(i,1:2));
    while out_of_map
        particles(i,1:2) = gen_random_particle([175,165], [24,200]);
        out_of_map = checkOccupancy(occ_grid, particles(i,1:2));
    end
    particles(i,4) = rand(1) * pi/18;
end
% Switch Spalte 2 mit 3 => x | y | z
z = particles(:,2);
y = particles(:,3);
particles(:,2) = y;
particles(:,3) = z;

% Plot x und z
plot(particles(:,1), particles(:,3), '.r')

part_struct.x = particles(:,1);
part_struct.y = particles(:,2);
part_struct.z = particles(:,3);
part_struct.orientation = particles(:,4);
part_struct.weights = particles(:,5);


% Hauptschleife
% TODO Kinect Daten simulieren, sodass Partikelfilter durchlaufen kann,
% dazu die robotPoses von oben verwenden und so umbauen, dass sie in die
% Partikelfilter Funktion reinpassen.
for f = 1 : 20
    distance_names = {'d1'; 'd2'; 'd3'; 'd4'; 'd5'; 'd6'; 'd7'; 'd8'; 'd9'; 'd10'};
    for t = 1 : N
        for d = 1 : length(distance_names)
            part_struct.distances(t).(distance_names{d}) = 0;
        end
    end
    
    % Partikel berechnen ihre Distanz zur Wand
    for k = 1 : N
        
        robot_pose = [part_struct.x(k), part_struct.z(k), part_struct.orientation(k)];
        intersectionPts = rayIntersection(occ_grid,robot_pose,angles,maxrange,0.7);
        for i = 1 : length(intersectionPts)
            if isnan(intersectionPts(i,1))
                % Abstand zur Wand ist immer gleich, deswegen ist die Hoehe (y) egal
                part_struct.distances(k).(distance_names{i}) = 0;
                
            else
                part_struct.distances(k).(distance_names{i}) = sqrt( (intersectionPts(i,1) - part_struct.x(k) )^2 ...
                    + (intersectionPts(i,2) - part_struct.z(k) )^2 );
            end
        end
    end
    
    %save('part_struct.mat', 'part_struct');
    
    toc
    % Partikelfilter
    load('ROI');
    %load('part_struct');
    resampled_particles = partikelfilter(part_struct, ROI);
    for i = 1 : N
        single_particle = resampled_particles(i);
        part_struct(i) = motionModel(single_particle);
        out_of_map = checkOccupancy(occ_grid, [part_struct.x(i), part_struct.z(i)]);
        while out_of_map
            temp_part = gen_random_particle([175,165], [24,200]);
            part_struct.x(i) = temp_part(1);
            part_struct.z(i) = temp_part(2);
            part_struct.orientation = rand(1) * pi/18;
            out_of_map = checkOccupancy(occ_grid, temp_part);
        end
    end
end