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
title({'Occupancy Grid', 'Skalierung: 26 Pixel entspricht 1 Meter'})
xlabel('X [Meter]')
ylabel('Y [Meter]')
legend off

% Simulate kinect data
q_range = randn(1) * 0.2;
% q_angle = randn(1) * 0.2;
% fov_deg = [-30, -25, -20, -10, -5, 0, 5, 10, 23, 30];
fov_deg = [30, 23, 10, 5, 0, -5, -10, -20, -25, -30];
% fov_deg = [60, 65, 70, 80, 85, 90, 95, 100, 113, 120]
%fov_deg = [120, 113, 100, 95, 85, 90, 80, 70, 65, 60] * -1;
% fov_deg = flipud(fov_deg)
% fov_deg = [30, 35, 40, 50, 55, 60, 65, 70, 83, 90];
fov = deg2rad(fov_deg);
%fov = [-0.5236,-0.4189,-0.3142,-0.2095,-0.1048,0.1046,0.2093,0.3140,0.4187,0.5234]; % +q_angle
% fov = fov + q_angle;
maxrange = 4;% + q_range;

% Init Partikel
N = 500; % Anzahl Partikel
particles = zeros(N,5);

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
    particles(i,3) = -1.7 + (1.7+0.8)*rand(1);
%     particles(i,4) = -pi + (pi+pi)*rand(1);
    particles(i,4) = (pi*2)/3;
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
legend(particle_plot, 'Initiale Partikel');

% Partikel in struct konvertieren
part_struct.x = particles(:,1);
part_struct.y = particles(:,2);
part_struct.z = particles(:,3);
part_struct.orientation = particles(:,4);
part_struct.weights = particles(:,5);
fieldnames = {'x'; 'y'; 'z'; 'orientation'; 'weights'};
%%
%------------------------------------------------------------------
%------------------------------------------------------------------
% NICHT DYNAMISCH!!!!!!!
% tic
% for r = 1:17
%     ROIs(r) = {load(['ROI', num2str(r)])};
%     % Calculate mean of roi segments
%     num_segments = 10;
%     min_angle = min(ROIs{1,r}.ROI.angle);
%     max_angle = max(ROIs{1,r}.ROI.angle);
%     step = (max_angle - min_angle) / num_segments;
%     point_index = 1;
%     segment_points = [ROIs{1,r}.ROI.x(1),ROIs{1,r}.ROI.y(1), ROIs{1,r}.ROI.z(1), ...
%                       ROIs{1,r}.ROI.dist(1), ROIs{1,r}.ROI.angle(1)];
%
%     for s = 1 : num_segments
%
%         for i = 1 : length(ROIs{1,r}.ROI.x)
%             if ROIs{1,r}.ROI.angle(i) < min_angle + step
%                 segment_points(point_index,:) = [ROIs{1,r}.ROI.x(i), ROIs{1,r}.ROI.y(i), ...
%                             ROIs{1,r}.ROI.z(i), ROIs{1,r}.ROI.dist(i), ROIs{1,r}.ROI.angle(i)];
%                 point_index = point_index + 1;
%             end
%         end
%
%         % Mitteln
%         segments(s,1) = mean(segment_points(:,1));
%         segments(s,2) = mean(segment_points(:,2));
%         segments(s,3) = mean(segment_points(:,3));
%
%         % Strecke von (0,0,0) 3D
%         segments(s,4) = sqrt( (segments(s,1))^2 + (segments(s,2))^2 + (segments(s,3))^2 );
%         % Winkel zu (0,0) 2D
%         segments(s,5) = atan2( segments(s,3),segments(s,1) );
%         point_index = 1;
%         min_angle = min_angle + step;
%         s =  s + 1;
%     end
%     ROIs{1,r}.segments = segments;
%     r
%     toc
% end
% save('ROIS', 'ROIs')

%%

% Hauptschleife
tic
load('ROIs')
toc
for h = 1 : length(ROIs)  % Hauptschleife
    
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
    
    % (4) Propagation aller Partikel durch Bewegungsmodell
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
    
    % Partikelfilter
    resampled_particles = partikelfilter(part_struct, ROIs{1, h}.segments);
    
    part_struct = resampled_particles;
    
    
    figure(1)
    hold on
    delete(particle_plot);
    particle_plot = plot(resampled_particles.x, resampled_particles.z, '*');      
    for i = 1 : N
        orientation_plot(i) = plot([resampled_particles.x(i), resampled_particles.x(i) + 0.5 * cos(resampled_particles.orientation(i))],...
            [resampled_particles.z(i), resampled_particles.z(i) + 0.5 * sin(resampled_particles.orientation(i))], 'g');
        
        legend off
    end
    title({'Partikelfilter', ['Iteration: ', num2str(h)], ['Partikel: ', ...
        num2str(N)]});
    legend([particle_plot, orientation_plot], {'Partikel', 'Orientierung'});
    pause(0.002);
    delete(orientation_plot);
    toc
end % Hauptschleife
disp('DONE')
toc