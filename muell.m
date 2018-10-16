%% eine woche frueher als 2018/10/16
%% INIT

clear all;
close all;
clc
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
figure(2)
pcshow(pc);
% Profillinie
figure(3)

subplot(2,1,1)
plot(point(:,1), point(:,3),'.')
grid on
subplot(2,1,2)
plot(roi(:,1), roi(:,3),'.r')
grid on


% ROI als .mat abspeichern mit Zeitstempel
data = struct('time', datestr(now), 'roi', roi);
save('data.mat', 'data');

%% Occupancy Grid > Map Creation(i)
clc
close all
figure(4)
grid = load('Vorgabe.txt');
plot(grid(:,1), grid(:,2), '.k');
tic
% schnitt1 = load('SS2017_BA_Stuber_SchnitteGang_Betr_Czaja/Schnitt1.txt');
% schnitt2 = load('SS2017_BA_Stuber_SchnitteGang_Betr_Czaja/Schnitt2.txt');
% schnitt3 = load('SS2017_BA_Stuber_SchnitteGang_Betr_Czaja/Schnitt3.txt');
% schnitt4 = load('SS2017_BA_Stuber_SchnitteGang_Betr_Czaja/Schnitt4.txt');
% schnitt5 = load('SS2017_BA_Stuber_SchnitteGang_Betr_Czaja/Schnitt5.txt');
% schnitt6 = load('SS2017_BA_Stuber_SchnitteGang_Betr_Czaja/Schnitt6.txt');

% size1 = size(schnitt1,1);
% size2 = size(schnitt2,1);
% size3 = size(schnitt3,1);
% size4 = size(schnitt4,1);
% size5 = size(schnitt5,1);
% size6 = size(schnitt6,1);
% GPS1 = schnitt1(1:100:size1,:);
% GPS2 = schnitt2(1:100:size2,:);
% GPS3 = schnitt3(1:100:size3,:);
% GPS4 = schnitt4(1:100:size4,:);
% GPS5 = schnitt5(1:100:size5,:);
% GPS6 = schnitt6(1:100:size6,:);
% save('Grundriss_GPS.mat', 'GPS1', 'GPS2', 'GPS3', 'GPS4', 'GPS5', 'GPS6')
load('Grundriss_GPS.mat')
% Plot
% figure(3)
% subplot(2,1,1)
% plot(1:size1,schnitt1(:,3),'.k');
% hold on;
% plot(1:size2,schnitt2(:,3),'.y');
% hold on;
% plot(1:size3,schnitt3(:,3),'.c');
% hold on;
% plot(1:size4,schnitt4(:,3),'.r');
% hold on;
% plot(1:size5,schnitt5(:,3),'.b');
% hold on;
% plot(1:size6,schnitt6(:,3),'.g');
% hold on;
% legend('schnitt 1','schnitt 2', 'schnitt 3','schnitt 4','schnitt 5','schnitt 6')
% % figure(1)
% % subplot(2,1,1)
% % plot(schnitt1(:,1), schnitt1(:,2), '.k')
% % subplot(2,1,2)
% % plot(GPS1(:,1),GPS1(:,2),'.k');
% % figure(2)
% % subplot(2,1,1)
% % plot(schnitt2(:,1), schnitt2(:,2), '.k')
% % subplot(2,1,2)
% % plot(GPS2(:,1),GPS2(:,2),'.k');
% % figure(3)
% % subplot(2,1,1)
% % plot(schnitt3(:,1), schnitt3(:,2), '.k')
% % subplot(2,1,2)
% % plot(GPS3(:,1),GPS3(:,2),'.k');
% % figure(4)
% % subplot(2,1,1)
% % plot(schnitt4(:,1), schnitt4(:,2), '.k')
% % subplot(2,1,2)
% % plot(GPS4(:,1),GPS4(:,2),'.k');
% % figure(5)
% % subplot(2,1,1)
% % plot(schnitt5(:,1), schnitt5(:,2), '.k')
% % subplot(2,1,2)
% % plot(GPS5(:,1),GPS5(:,2),'.k');
% % figure(6)
% % subplot(2,1,1)
% % plot(schnitt6(:,1), schnitt6(:,2), '.k')
% % subplot(2,1,2)
% % plot(GPS6(:,1),GPS6(:,2),'.k');

%
figure(1)
plot(schnitt1(:,1), schnitt1(:,2), '.k')
hold on
plot(schnitt2(:,1), schnitt2(:,2), '.y')
hold on
plot(schnitt3(:,1), schnitt3(:,2), '.c')
hold on
plot(schnitt4(:,1), schnitt4(:,2), '.r')
hold on
plot(schnitt5(:,1), schnitt5(:,2), '.b')
hold on
plot(schnitt6(:,1), schnitt6(:,2), '.g')
hold on
legend('schnitt 1','schnitt 2', 'schnitt 3','schnitt 4','schnitt 5','schnitt 6')
figure(2)
plot(GPS1(:,1),GPS1(:,2),'.k');
hold on
plot(GPS2(:,1),GPS2(:,2),'.y');
hold on
plot(GPS3(:,1),GPS3(:,2),'.c');
hold on
plot(GPS4(:,1),GPS4(:,2),'.r');
hold on
plot(GPS5(:,1),GPS5(:,2),'.b');
hold on
plot(GPS6(:,1),GPS6(:,2),'.g');
hold on
legend('schnitt 1','schnitt 2', 'schnitt 3','schnitt 4','schnitt 5','schnitt 6')

%% 2018/10/16
f = openfig('GPS_all.fig');
H = findobj(f,'type', 'line')
x = get(H,'xdata');
y = get(H,'ydata');
% close all
% plot([x{:,:}],[y{:,:}],'.b');
% axis equal

%% aus 3D Punktwolke floorplan erstellen ...

% load Data
% floorplan = load('HM_Karlstrasse_F8100_OG3_mod.map-1.txt');
% 
% 
% 
% floorplan(:,3) = 0;
% % Daten anzeigen
% for i = 1: length(floorplan)
%     floorplan(i,1) = floorplan(i,1)/1000;
%     floorplan(i,2) = floorplan(i,2)/1000;
% end
% 
% pcshow([floorplan(:,1), floorplan(:,2), floorplan(:,3)]);
% title('Occupancy Grid from Pointcloud');
% grid off;
% xlabel('X [meters]');
% ylabel('Y [meters]');
% view(2);
% hold on
% 
% % floorplan um min verschieben
% floorplan(:,1) = -min(floorplan(:,1)) + floorplan(:,1);
% floorplan(:,2) = -min(floorplan(:,2)) + floorplan(:,2);
% 
% % grid size abfragen
% max_x = max(floorplan(:,1));
% min_x = min(floorplan(:,1));
% max_y = max(floorplan(:,2));
% min_y = min(floorplan(:,2));
% 
% % Definition des Occupancy-Grids
% map = robotics.BinaryOccupancyGrid((round(max_x)+1),(round(max_y)+1), 5);
% 
% % projektion auf xy coordinaten
% XY = floorplan(:,1:2);
% 
% % for i=1 : 100 %length(XY)
% %xy = XY(i,:);
% %setOccupancy(map,xy, 1);
% %show(map);
% % end
% % occ grid erstellen
% setOccupancy(map, XY, ones(length(XY),1));
% %inflate(map,0.2)
% figure(2);
% show(map);


