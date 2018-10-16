%% INIT

clear all;
close all;
clc
GPS = load('Grundriss_GPS.mat');
vorgabe = load('Vorgabe.txt');
imaqreset
%% Data Aquisition (i)
colorDevice = imaq.VideoDevice('kinect',1);
depthDevice = imaq.VideoDevice('kinect',2);

step(colorDevice);
step(depthDevice);

colorImage = step(colorDevice);
depthImage = step(depthDevice);
% Bild von Kinect streamen (1 Frame)
ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);

% Fuer mehrere Frames Player verwenden
% player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
% 	'VerticalAxis','y','VerticalAxisDir','down');
% 
% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');
% 
% for i = 1:50
%    colorImage = step(colorDevice);  
%    depthImage = step(depthDevice);
%  
%    ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);

%    view(player,ptCloud);
% end

% Kamera Objekt wieder freigeben
release(colorDevice);
release(depthDevice);
%Pointcloud als ACSII PLY Datei abspeichern
pcwrite(ptCloud, 'scene.ply','Encoding','ascii');

%% Pointcloud einlesen und bearbeiten

pc = pcread('scene.ply');
square = 0.1; % ROI mit 20cm
roi = zeros(1,3);
index = 1;
% Alle Punkte in der Pointcloud abspeichern und 20 quadratcentimeter grosse ROI speichern
tic
for i = 1:pc.Count
   point(i,:) = pc.Location(i,:);
   if (point(i,2) >= (-square) && point(i,2) <= square) && (point(i,1) >= (-square) && point(i,1) <= square)
       roi(index,:) = pc.Location(i,:);
       index = index + 1;
   end
end
toc

% Pointcloud anzeigen
figure(1)
pcshow(pc);
% Profillinie
figure(2)
subplot(2,1,1)
plot(point(:,1), point(:,3),'.')
title('Profillinie gesamt')
grid on
subplot(2,1,2)
plot(roi(:,1), roi(:,3),'.r')
title('Profillinie ROI 20cm^2')
grid on

% ROI als .mat abspeichern mit Zeitstempel
data = struct('time', datestr(now), 'roi', roi);
save('data.mat', 'data');

%% Occupancy Grid > Map Creation(i)
% Grundriss
figure(3)
plot(vorgabe(:,1), vorgabe(:,2), '.k');
title('Grundriss Vorgabe')

% GPS
figure(4)
plot(GPS.GPS1(:,1),GPS.GPS1(:,2),'.k');
hold on
plot(GPS.GPS2(:,1),GPS.GPS2(:,2),'.k');
hold on
plot(GPS.GPS3(:,1),GPS.GPS3(:,2),'.k');
hold on
plot(GPS.GPS4(:,1),GPS.GPS4(:,2),'.k');
hold on
plot(GPS.GPS5(:,1),GPS.GPS5(:,2),'.k');
hold on
plot(GPS.GPS6(:,1),GPS.GPS6(:,2),'.k');
hold on
%legend('schnitt 1','schnitt 2', 'schnitt 3','schnitt 4','schnitt 5','schnitt 6')
title('GPS')
axis equal

%%
clc
f = openfig('GPS_all.fig');
H = findobj(f,'type', 'line')
x = get(H,'xdata');
y = get(H,'ydata');
close all
plot([x{:,:}],[y{:,:}],'.b');
axis equal






