%% load experiment data from csv
close all;
filename = '20240123_drvie-data.csv';
opts = detectImportOptions(filename);
preview(filename,opts)
% opts.SelectedVariableNames = [1:4];
exp_datas = readmatrix(filename);
time = exp_datas(:,1);
centroid_x = exp_datas(:,2);
centroid_y = exp_datas(:,3);
frwheel_x = exp_datas(:,4);
frwheel_z = exp_datas(:,5);
rrwheel_x = exp_datas(:,6);
floor_x = exp_datas(:,7);
floor_z = exp_datas(:,8);
velocity = exp_datas(:,9);

%% index
idx = time>-1;
idx = time>36 & time<39;

%% fft
idx = time>36.95 & time<37.23;
noisy_data = centroid_x(idx);
detrended_noisy_data = detrend(noisy_data);
noisy_time = time(idx);

sampling_freq = 1/0.02;
signal_length = length(detrended_noisy_data);
fft_noisy_data = fft(detrended_noisy_data);

P2 = abs(fft_noisy_data/signal_length);
P1 = P2(1:signal_length/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = sampling_freq/signal_length*(0:(signal_length/2));
drawer(f,P1,"Frequency [Hz]","One-sided spectrum"+newline+"|P1(f)|",fs,'FFT');

%% drawing datas for checking
% drawer(x,y,xlabel,ylabel,fontsize,filename)
% centroid
fs = 10;
drawer(time(idx),-1*centroid_x(idx),"Time [s]","Centroid Longitudinal"+newline+"Acceleration [m/s^2]",fs,'centroid_x');
drawer(time(idx),-1*centroid_y(idx),"Time [s]","Centroid Vertical"+newline+"Acceleration [m/s^2]",fs,'centroid_y');

% wheel longitudinal
drawer(time(idx),-1*frwheel_x(idx),"Time [s]","Front Right Wheel"+newline+"Longitudinal"+newline+"Acceleration [m/s^2]",fs,'frwheel_x');
drawer(time(idx),-1*rrwheel_x(idx),"Time [s]","Rear Right Wheel"+newline+"Longitudinal"+newline+"Acceleration [m/s^2]",fs,'rrwheel_x');

% wheel vertical
drawer(time(idx),frwheel_z(idx),"Time [s]","Front Right Wheel"+newline+"Vertical"+newline+"Acceleration [m/s^2]",fs,'frwheel_z');

% floor
drawer(time(idx),-1*floor_x(idx),"Time [s]","Floor Longitudinal"+newline+"Acceleration [m/s^2]",fs,'floor_x');
drawer(time(idx),floor_z(idx),"Time [s]","Floor Vertical"+newline+"Acceleration [m/s^2]",fs,'floor_z');

% velocity
drawer(time(idx),velocity(idx),"Time [s]","Driving Velocity"+newline+"Acceleration [m/s^2]",fs,'velocity');

%% find peaks
current_time = time(idx);
current_centroid_x =centroid_x(idx);
pks = findpeaks(-1*centroid_x(idx),1/50);
hold on;
scatter(current_time(ismember(current_centroid_x,-pks)),-1*current_centroid_x(ismember(current_centroid_x,-pks)))