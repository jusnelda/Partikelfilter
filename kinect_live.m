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

%% Kinect-Live-Daten vom 11.12.2018
% Nehmen Daten von der unteren rechten Wand bis zur oberen rechten Wand auf
% und den Knick der nach links verl�uft, f�r die Eindeutigkeit
% Format der Dateien sind .ply Dateien
clear all;
close all;
clc

for i=1:17
    str = ['pc', num2str(i), '.ply'];
    filename(i) = {str};
end

%% Pointcloud einlesen und bearbeiten
for i = 1 : length(filename)
   pc = pcread(cell2mat(filename(i)));
   figure(i)
   hold on
   title(['Bild #', num2str(i)]);
   p = pcshow(pc);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   view(0, -80); 
%    pause(0.4);
%     delete(p);
end

%%
square = 0.1;
% max_height = 0.58; % Fuer wand rechts daten
max_height = 0.8;
roi = zeros(1,3);
index = 1;
tic
disp(['startTimer: ', datestr(now)])
for i = 1 : length(filename)
    pc = pcread(cell2mat(filename(i)));
    pc_filtered = pc;
    roi = zeros(1,3);
    index = 1;
    % Alle Punkte in der Pointcloud abspeichern und 20 quadratcentimeter grosse ROI speichern
    for k = 1 : pc.Count
        point(k,:) = pc.Location(k,:);
        % Nach Y-Hoehe filtern
        if (point(k,2) <= (max_height)) %||point(k,2) >= max_height  %&& (point(i,1) >= (-square) && point(i,1) <= square)
            roi(index,:) = pc.Location(k,:);
%             pc_filtered.Location(k,2) = 0;

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
%     ROI = struct('time', datestr(now), 'x', roi(:,1), 'y', roi(:,2), 'z', roi(:,3), 'dist', roi(:,4), 'angle', roi(:,5));
%     save(['ROI', num2str(i), '.mat'], 'ROI');
    figure(2)
    pp = polarplot(roi(:,5), roi(:,4), '.');
    disp(num2str(i));
    pause(0.02);
    delete(pp);
    
end % for length filename
toc



