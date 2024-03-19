%% load experiment data from csv
filename = '20240123_drvie-data.csv';
opts = detectImportOptions(filename);
preview(filename,opts)
% opts.SelectedVariableNames = [1:4];
exp_datas = readmatrix(filename);
time = exp_datas(:,1);
centroid_x = exp_datas(:,2);
frwheel_x = exp_datas(:,3);
frwheel_z = exp_datas(:,4);
rrwheel_x = exp_datas(:,5);
floor_x = exp_datas(:,6);
floor_z = exp_datas(:,7);

%% index
idx = time>-1;
idx = time>36 & time<39;

%% drawing datas for checking
close all;
% drawer(x,y,xlabel,ylabel,fontsize,filename)
% centroid
fs = 10;
drawer(time(idx),-1*centroid_x(idx),"Time [s]","Centroid Longitudinal"+newline+"Acceleration [m/s^2]",fs,'centroid_x');

% wheel longitudinal
drawer(time(idx),-1*frwheel_x(idx),"Time [s]","Front Right Wheel"+newline+"Longitudinal"+newline+"Acceleration [m/s^2]",fs,'frwheel_x');
drawer(time(idx),-1*rrwheel_x(idx),"Time [s]","Rear Right Wheel"+newline+"Longitudinal"+newline+"Acceleration [m/s^2]",fs,'rrwheel_x');

% wheel vertical
drawer(time(idx),frwheel_z(idx),"Time [s]","Front Right Wheel"+newline+"Vertical"+newline+"Acceleration [m/s^2]",fs,'frwheel_z');

% floor
drawer(time(idx),-1*floor_x(idx),"Time [s]","Floor Longitudinal"+newline+"Acceleration [m/s^2]",fs,'floor_x');
drawer(time(idx),floor_z(idx),"Time [s]","Floor Vertical"+newline+"Acceleration [m/s^2]",fs,'floor_z');