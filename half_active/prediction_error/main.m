clear;

iteration_num = 10000;

load_dir = "preview_datas";
listing = dir(load_dir+"/*.mat");
len = length(listing);

angle_errors = zeros(len*iteration_num,1);
height_errors = zeros(len,iteration_num);

for i = 1:len    
    file = load(load_dir + "/" + listing(i).name);
    vertices = file.vertices;   
    for k = 1:iteration_num
        [prev_profile, angle_d, predicted_height] = previewing(vertices);

        angle_errors(k+(i-1)*iteration_num,1) = angle_d-16;
        height_errors(i,k) = predicted_height;
        if mod(k,100) == 0
            display(i+":"+k);
        end
    end
    height_errors(i,:) = height_errors(i,:) - mean(height_errors(i,:));
end
height_errors = reshape(height_errors, size(angle_errors));

trend_error_var = var(angle_errors);
height_error_var = var(height_errors);

save("variances.mat","trend_error_var","height_error_var","angle_errors","height_errors")