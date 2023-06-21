close all;
load("variances.mat")

a_fig = figure("Name","height_errors","Position",[680,693,467,185]);
plot(height_errors,"Color","#0000ff"), grid on;
xlabel("N = 23000");
ylabel("Height Estimation Error [m]");
fontname(a_fig,"Times New Roman");
fontsize(a_fig,9.5,"points");

b_fig = figure("Name","angle_errors","Position",[680,693,467,185]);
[C,I] = sort(rand(length(angle_errors),1));
plot(angle_errors(I),"Color","#0000ff"), grid on;
xlabel("N = 23000");
ylabel("Angle Estimation Error [rad]");
fontname(b_fig,"Times New Roman");
fontsize(b_fig,9.5,"points");

c_fig = figure("Name","sensor_vars","Position",[680,693,467,185]);
[~,I] = sort(rand(length(sensor_vars),1));
plot(sensor_vars(I),"Color","#0000ff"), grid on;
xlabel("N = 23000");
ylabel("Sensor Noise Variances");
fontname(c_fig,"Times New Roman");
fontsize(c_fig,9.5,"points");


load_dir = "preview_datas";
listing = dir(load_dir+"/*.mat");
len = length(listing);
V = 50*1000/3600; % m/s
ts = 0.001;
fs = 1/ts;
file = load(load_dir + "/" + listing(1).name);
vertices = file.vertices;   
[prev_profile, angle_d, predicted_height] = previewing(vertices);
[~,ia,~]=unique(prev_profile(1,:));
prev_profile = prev_profile(:,ia);
prev_profile = rmmissing(prev_profile,2);
T = (prev_profile(1,end) - prev_profile(1,1))/V;
t = 0:ts:T;
X = prev_profile(2,:);
X = interp1((prev_profile(1,:)-prev_profile(1,1))./V,X,t,"linear");
Y=highpass(X,6,fs);
% Y=highpass(X,25,fs);
figure("Position",[680,693,467,185])
plot(t.*V+5.06,X,"LineWidth",2,"Color","#0000ff"), grid on, hold on;
plot(t.*V+5.06,Y,"LineWidth",2,"Color","#ff0000")
ylabel("Displacement [m]");
legend("Raw","HPF")
fontname(gca,"Times New Roman");
fontsize(gca,9.5,"points");