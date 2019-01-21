% %% INIT
% 
% clear all;
% close all;
% clc
% imaqreset
% 
% % Data Aquisition
% colorDevice = imaq.VideoDevice('kinect',1);
% depthDevice = imaq.VideoDevice('kinect',2);
% 
% colorImage = step(colorDevice);
% depthImage = step(depthDevice);
% ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
% 
% 
% 
% step(colorDevice);
% step(depthDevice);

% t0 = clock;
% file_index = 1;
% while etime(clock, t0) < 120
%     if (mod(etime(clock,t0),3) == 0) % alle 3 sekunden
%         disp(['Getting new frame: ', num2str(etime(clock,t0))]);
%         tic
%         colorImage = step(colorDevice);
%         depthImage = step(depthDevice);
%         ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
%         str = ['scene', num2str(etime(clock,t0)), '.ply'];
%         filename(file_index) = {str};
%         pcwrite(ptCloud, str, 'Encoding','ascii');
%         file_index = file_index + 1;
%         toc
%     end
% end
% 
% % Kamera Objekt wieder freigeben
% release(colorDevice);
% release(depthDevice);

%% Kinect-Live-Daten vom 11.12.2018
% Nehmen Daten von der unteren rechten Wand bis zur oberen rechten Wand auf
% und den Knick der nach links verl�uft, f�r die Eindeutigkeit
% Format der Dateien sind .ply Dateien
clear all;
close all;
clc
addpath('ROI_Wand_Rechts')
for i=1:37
    str = ['ROI_Wand_Rechts/', 'pc', num2str(i), '.ply'];
    filename(i) = {str};
end

%% Pointcloud einlesen und anzeigen
for i = 1 : length(filename)
    pc = pcread(cell2mat(filename(i)));
    
    figure(i)
    hold on
    title(['Bild #', num2str(i)]);
    p = pcshow(pc);
    xlabel('X [Meter]');
    ylabel('Y [Meter]');
    zlabel('Z [Meter]');
    grid on
    %view(0, -80);
    figure(2)
    pc = pcdenoise(pc); %interesting
    pcshow(pc);
  pause(0.4);
%     %     delete(p);
end

%%
square = 0.1;
max_height = 0.58; % Fuer wand rechts daten
% max_height = 0.8; % KinectAufnahmeII
roi = zeros(1,3);
index = 1;
tic
disp(['Start Timer: ', datestr(now)])
for i = 1 : length(filename)
    pc = pcread(cell2mat(filename(i)));
    pc_filtered = pc;
    roi = zeros(1,3);
    point = zeros(pc.Count,3);
    index = 1;
    % Alle Punkte in der Pointcloud abspeichern und 20 quadratcentimeter grosse ROI speichern
    for k = 1 : pc.Count
        color_pc = pc.Color;
        point(k,:) = pc.Location(k,:);
        % Nach Y-Hoehe filtern
        if (point(k,2) <= (max_height)) %||point(k,2) >= max_height  %&& (point(i,1) >= (-square) && point(i,1) <= square)
            roi(index,:) = pc.Location(k,:);
            index = index + 1;
        else
            point(k,:) = [];
            color_pc(k,:) = [];
        end
    end % for pointcloud
    %Creating a new point-cloud
    new_pc = pointCloud(point, 'Color', color_pc);
    %Saving the new point-cloud
    %pcwrite (new_pc , 'pc_filtered.ply');
    
    % Polarkoordinaten berechnen fuer roi
    for n = 1 : length(roi)
        % Strecke von (0,0,0) 3D
        roi(n,4) = sqrt( (roi(n,1))^2 + (roi(n,2))^2 + (roi(n,3))^2 );
        % Winkel zu (0,0) 2D
        roi(n,5) = atan2( roi(n,3),roi(n,1) );
    end
    % ROI als .mat abspeichern mit Zeitstempel
    %     ROI = struct('time', datestr(now), 'x', roi(:,1), 'y', roi(:,2), 'z', roi(:,3), 'dist', roi(:,4), 'angle', roi(:,5));
    %     save(['ROI', num2str(i), '.mat'], 'ROI');
    figure(2)
    set(gcf, 'units', 'normalized', 'outerposition',[0 0 1 1])
    subplot(1,2,1)    
    showPointCloud(new_pc);
    title({'Gefilterte Punktwolke', ['Bild #', num2str(i)]});
    xlabel('X [Meter]');
    ylabel('Y [Meter]');
    zlabel('Z [Meter]');
    view(0, -80);
    subplot(1,2,2)
    pp = polarplot(roi(:,5), roi(:,4), '.');
    title('Polarplot der gefilterten Punktwolke');
    %disp(num2str(i));
    if i < length(filename)
        pause(0.02);
        delete(pp);
    end
toc    
end % for length filename
toc



