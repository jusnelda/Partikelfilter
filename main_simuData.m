%% Occupancy grid from png von mathworks
clc;
close all;
clear all;
format long g;
tic
% Import Image
%filepath = fullfile('\\R311-IMAC25\WS2017_PrjNav_Data\3_Gruppe\HM_Karlstrasse_F8100_OG3_mod.png');
% GPS Bild
%filepath = fullfile('Bilder/GPS_all_black.png');
%load('walls_black')
% Raum230 Bild
filepath = fullfile('Bilder/Room.PNG');
load('room_black')

%filepath = fullfile('D:\Partikelfilter\Partikelfilter/Bilder/GPS_all_black.png');

image = imread(filepath);

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 245;

% Use black and white image as matrix input for binary occupancy grid
occ_grid = robotics.OccupancyGrid(bwimage,25); % 26 Pixel == 1 Meter
bin_occ_grid = robotics.BinaryOccupancyGrid(bwimage,25); % 6,5 Pixel  == 0.25 Meter 
max_x_lim = occ_grid.XWorldLimits;
max_y_lim = occ_grid.YWorldLimits;
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
% save('room_black', 'walls');

%%
% Plot Occupancy Grid
figure(1);
set(gcf, 'units', 'normalized', 'outerposition',[0 0 0.5 0.5])
hold on
show(occ_grid);

% INITS fuer Hauptschleife
% Simulate robot poses
[x_s, y_s] = ginput(2);
% start= [x_s(1), y_s(1)];
% fin= [x_s(2), y_s(2)];
% For faster testing - predefined start and end
% start = [28.8462450592885,6.90148221343874];
% fin = [31.3693675889328,13.5803359683795];
robotPoses = data_simu(start,fin);

% Simulate kinect data
q_range = randn(1) * 0.4;
q_angle = randn(1) * 0.2;
fov_deg = [-30, -25, -20, -10, -5, 0, 5, 10, 23, 30];
fov = deg2rad(fov_deg);
%fov = [-0.5236,-0.4189,-0.3142,-0.2095,-0.1048,0.1046,0.2093,0.3140,0.4187,0.5234]; % +q_angle
% fov = fov + q_angle;
maxrange = 5 + q_range;

% Init Partikel
N = 500; % Anzahl Partikel
particles = zeros(N,5);
% TODO - y = Hoehe der Kinect ueber dem Boden
% particles(:,3) = Hoehe
particles(:,5) = 1/N;
% Partikel generieren und pruefen ob sie einen validen Punkt auf der Karte
% haben
for i = 1 : N
    particles(i,1:2) = gen_random_particle(max_x_lim, max_y_lim);
    out_of_map = checkOccupancy(occ_grid, particles(i,1:2));
    while out_of_map
        particles(i,1:2) = gen_random_particle(max_x_lim, max_y_lim);
        out_of_map = checkOccupancy(occ_grid, particles(i,1:2));
    end
particles(i,4) = -pi + (pi+pi)*rand(1);
end
% Switch Spalte 2 mit 3 => X | Y | Z
z = particles(:,2);
y = particles(:,3);
particles(:,2) = y;
particles(:,3) = z;

% Plot X und Z ( Y ist Hoehe )
set(gcf, 'units', 'normalized', 'outerposition',[0 0 1 1]);
title({'Particlefilter', 'Iteration: 0'});
particle_plot = plot(particles(:,1), particles(:,3), '.r');

% Partikel in struct konvertieren
part_struct.x = particles(:,1);
part_struct.y = particles(:,2);
part_struct.z = particles(:,3);
part_struct.orientation = particles(:,4);
part_struct.weights = particles(:,5);
fieldnames = {'x'; 'y'; 'z'; 'orientation'; 'weights'};
%
for r = 1:20
    ROIs(r) = {load(['ROI', num2str(r)])};
end
%%

% Hauptschleife

for h = 1 : length(ROIs) % length(robotPoses) % Hauptschleife (spaeter while true)
    
    % Ersten Partikel zum testen immer auf aktuelle Pose setzen
%     part_struct.x(1) = robotPoses(h,1); 
%     part_struct.z(1) = robotPoses(h,2);
    % Simulierte Kinect Daten generieren (intsectionPts der robotPoses mit
    % der Wand)
%     [kinect_data, plots] = data_simu_kinect(occ_grid,robotPoses(h,:),fov,maxrange); % dummy data
    % Datenstruktur zum abspeichern der 10 vergleichbaren Distanzen
    distance_names = {'d1'; 'd2'; 'd3'; 'd4'; 'd5'; 'd6'; 'd7'; 'd8'; 'd9'; 'd10'};
    for t = 1 : N
        for d = 1 : length(distance_names)
            part_struct.distances(t).(distance_names{d}) = 0;
        end % for-loop Benennung der distances Unterspalten
    end % for-loop benennung der distances Spalte im part_struct
    
    % Partikel berechnen ihre Distanz zur Wand
    for k = 1 : N
        
        part_pose = [part_struct.x(k), part_struct.z(k), part_struct.orientation(k)];
        intersectionPts = rayIntersection(occ_grid,part_pose,fov,maxrange,0.7);
        
        for i = 1 : length(intersectionPts)
            if isnan(intersectionPts(i,1))
                % Abstand zur Wand ist immer gleich, deswegen ist die Hoehe (y) egal
                part_struct.distances(k).(distance_names{i}) = 0;                
            else
                part_struct.distances(k).(distance_names{i}) = ...
                    sqrt( (intersectionPts(i,1) - part_struct.x(k) )^2 ...
                        + (intersectionPts(i,2) - part_struct.z(k) )^2 );
            end % if intersectionPts isnan
        end % for-loop ueber alle intersectionPts
    end % for-loop ueber alle Partikel
    
    % Partikelfilter
    % ROI = load('ROI');
   
    
    resampled_particles = partikelfilter(part_struct, ROIs{1, h}.ROI);

    % resampled_particles = partikelfilter(part_struct, kinect_data);
    part_struct = resampled_particles;
%     part_struct.x(1) = robotPoses(h,1); part_struct.z(1) = robotPoses(h,2);
    for i = 1 : N
        single_particle = [part_struct.x(i), part_struct.z(i), part_struct.orientation(i)];
        % Motion Model
        [part_struct.x(i), part_struct.z(i)] = motionModel(single_particle);
        if part_struct.x(i) < max_x_lim(2) && part_struct.z(i) < max_y_lim(2)           
            out_of_map = checkOccupancy(occ_grid, [part_struct.x(i), part_struct.z(i)]);
        else
            out_of_map = true;
        end
        while out_of_map
            temp_part = gen_random_particle(max_x_lim, max_y_lim);
            part_struct.x(i) = temp_part(1);
            part_struct.z(i) = temp_part(2);
            out_of_map = checkOccupancy(occ_grid, temp_part);
        end

    end
    figure(1)
    delete(particle_plot);
    particle_plot = plot(resampled_particles.x, resampled_particles.z, '.b');
    title({'Particlefilter', ['Iteration: ', num2str(h)], ['Particles: ', ...
            num2str(N), ' | Range Noise: ', num2str(q_range), ' | Angle Noise: '...
            num2str(q_angle)]});
%     legend([particle_plot, plots.intsections, plots.robot, plots.nonrays, ...
%             plots.rays], {'Particles', 'Collision Points', 'Robot Pose', ...
%             'Rays (No Collision)', 'Rays (Collison)'});

    toc
end % Hauptschleife
disp('DONE')
toc