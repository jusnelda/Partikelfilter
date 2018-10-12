
clc;
close all;
clear all;
%% aus 3D Punktwolke floorplan erstellen ...

% load Data
floorplan = load('HM_Karlstrasse_F8100_OG3_mod.map-1.txt');



floorplan(:,3) = 0;
% Daten anzeigen
for i = 1: length(floorplan)
    floorplan(i,1) = floorplan(i,1)/1000;
    floorplan(i,2) = floorplan(i,2)/1000;
end

pcshow([floorplan(:,1), floorplan(:,2), floorplan(:,3)]);
title('Occupancy Grid from Pointcloud');
grid off;
xlabel('X [meters]');
ylabel('Y [meters]');
view(2);
hold on

% floorplan um min verschieben
floorplan(:,1) = -min(floorplan(:,1)) + floorplan(:,1);
floorplan(:,2) = -min(floorplan(:,2)) + floorplan(:,2);

% grid size abfragen
max_x = max(floorplan(:,1));
min_x = min(floorplan(:,1));
max_y = max(floorplan(:,2));
min_y = min(floorplan(:,2));

% Definition des Occupancy-Grids
map = robotics.BinaryOccupancyGrid((round(max_x)+1),(round(max_y)+1), 5);

% projektion auf xy coordinaten
XY = floorplan(:,1:2);

% for i=1 : 100 %length(XY)
%xy = XY(i,:);
%setOccupancy(map,xy, 1);
%show(map);
% end

% occ grid erstellen
setOccupancy(map, XY, ones(length(XY),1));


%inflate(map,0.2)

figure(2);
show(map);



%% Occupancy grid from png von mathworks
close all
clc
% Import Image
%filepath = fullfile('\\R311-IMAC25\WS2017_PrjNav_Data\3_Gruppe\HM_Karlstrasse_F8100_OG3_mod.png');
filepath = fullfile('/Users/justine/Documents/7.Semester/Partikelfilter/GPS_all_edited.png');
image = imread(filepath);

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 255;

% Use black and white image as matrix input for binary occupancy grid
occ_grid = robotics.BinaryOccupancyGrid(bwimage);
figure(3)
show(occ_grid)







