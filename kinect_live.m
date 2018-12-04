%% INIT

clear all;
close all;
clc
imaqreset

% Data Aquisition (i)
colorDevice = imaq.VideoDevice('kinect',1);
depthDevice = imaq.VideoDevice('kinect',2);

step(colorDevice);
step(depthDevice);
% colorImage = step(colorDevice);
% depthImage = step(depthDevice);
% ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);


t0 = clock;
file_index = 1;
while etime(clock, t0) < 120
   if (mod(etime(clock,t0),3) == 0) % alle 3 sekunden
       disp(['Getting new frame: ', num2str(etime(clock,t0))]);
       tic
       colorImage = step(colorDevice);
       depthImage = step(depthDevice);
       ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
       str = ['scene', num2str(etime(clock,t0)), '.ply'];
       filename(file_index) = {str};
       pcwrite(ptCloud, str, 'Encoding','ascii');
       file_index = file_index + 1;
       toc
   end
end

% Kamera Objekt wieder freigeben
release(colorDevice);
release(depthDevice);

%% Pointcloud einlesen und bearbeiten
for i = 1 : length(filename)
   pc = pcread(cell2mat(filename(i)));
   figure(i)
   pcshow(pc);
end

%%
square = 0.1; % ROI mit 20cm
roi = zeros(1,3);
index = 1;
tic
for i = 1 : length(filename)
    pc = pcread(cell2mat(filename(i)));
    roi = zeros(1,3);
    index = 1;
    % Alle Punkte in der Pointcloud abspeichern und 20 quadratcentimeter grosse ROI speichern
    for k = 1 : pc.Count
        point(k,:) = pc.Location(k,:);
        % Nach Y-Hoehe filtern
        if (point(k,2) >= (-square) && point(k,2) <= square) %&& (point(i,1) >= (-square) && point(i,1) <= square)
            roi(index,:) = pc.Location(k,:);
            index = index + 1;
        end
    end % for pointcloud
    % Polarkoordinaten berechnen fuer roi
    for n = 1 : length(roi)
        % Strecke von (0,0,0) 3D
        roi(n,4) = sqrt( (roi(n,1))^2 + (roi(n,2))^2 + (roi(n,3))^2 );
        % Winkel zu (0,0) 2D
        roi(n,5) = atan2( roi(n,3),roi(n,1) );
    end
    % ROI als .mat abspeichern mit Zeitstempel
    ROI = struct('time', datestr(now), 'x', roi(:,1), 'y', roi(:,2), 'z', roi(:,3), 'dist', roi(:,4), 'angle', roi(:,5));
    save(['ROI', num2str(i), '.mat'], 'ROI');
    
end % for length filename
toc



