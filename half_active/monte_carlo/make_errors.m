load("../prediction_error/variances.mat")

rng(0,'twister');
trend_std = sqrt(trend_error_var);
height_std = sqrt(height_error_var);
noise_std = sqrt(sensor_noise_var);

trend_error_list = round(deg2rad(trend_std)*randn(1,200),1);
height_error_list = round(height_std*randn(1,200),3);
sensor_noise_list = round(noise_std*randn(1,200),3);

idx = randi(200,[100,500,2]);

plane_errors_set = zeros([100,500,2]);

for i = 1:80
    plane_errors_set(i,:,1) = trend_error_list(reshape(idx(i,:,1),1,500));
    plane_errors_set(i,:,2) = height_error_list(reshape(idx(i,:,2),1,500));
end
save("data_set.mat","plane_errors_set","trend_error_list","height_error_list","sensor_noise_list")