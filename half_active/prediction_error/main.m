monte_iter = 10000;


load_dir = "preview_datas";
listing = dir(load_dir+"/*.mat");
len = length(listing);

angle_errors = zeros(len*monte_iter,1);
height_errors = zeros(len,monte_iter);
vertices_list = [];

for i = 1:len
    file = load(load_dir + "/" + listing(i).name);
    vertices = file.vertices;
    vertices_list(i) = vertices;
end

for i = 1:len       
    for k = 1:monte_iter
        vertices = vertices_list(i);
        [prev_profile, angle_d, predicted_height] = previewing(vertices);

        angle_errors(k+(i-1)*monte_iter,1) = angle_d-16;
        height_errors(i,k) = predicted_height;
        if mod(k,100) == 0
            display(i+":"+k);
        end
    end
    height_errors(i,:) = height_errors(i,:) - mean(height_errors(i,:));
end
height_errors = reshape(height_errors, size(angle_errors));