clear;

iteration_num = 1000;

load_dir = "preview_datas";
listing = dir(load_dir+"/*.mat");
len = length(listing);

angle_errors = zeros(len*iteration_num,1);
height_errors = zeros(len,iteration_num);
sensor_vars = zeros(len*iteration_num,1);

V = 50*1000/3600; % m/s
ts = 0.001;
fs = 1/ts;


for i = 1:len    
    file = load(load_dir + "/" + listing(i).name);
    vertices = file.vertices;   
    for k = 1:iteration_num
        [prev_profile, angle_d, predicted_height] = previewing(vertices);
        [~,ia,~]=unique(prev_profile(1,:));
        prev_profile = prev_profile(:,ia);
        prev_profile = rmmissing(prev_profile,2);
        T = (prev_profile(1,end) - prev_profile(1,1))/V;
        t = 0:ts:T;
        X = prev_profile(2,:);
        X = interp1((prev_profile(1,:)-prev_profile(1,1))./V,X,t,"linear");
        Y=highpass(X,25,fs);
        sensor_vars(k+(i-1)*iteration_num,1) = var(Y);

        angle_errors(k+(i-1)*iteration_num,1) = angle_d-16;
        height_errors(i,k) = predicted_height;
        
        clear prev_profile angle_d predicted_heigh;
        if mod(k,100) == 0
            display(i+":"+k);
        end
    end
    height_errors(i,:) = height_errors(i,:) - mean(height_errors(i,:));
end
height_errors = reshape(height_errors, size(angle_errors));

trend_error_var = var(angle_errors);
height_error_var = var(height_errors);
sensor_noise_var = mean(sensor_vars);

save("variances.mat","trend_error_var","height_error_var","sensor_noise_var","angle_errors","height_errors","sensor_vars")