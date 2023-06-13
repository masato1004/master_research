monte_iter_num = 500;

ld_list = [0.25, 0.5, 1, 1.5, 2];
max_z0_list = [0.02, 0.03, 0.04, 0.05, 0.08];

pitch_inte_list_onlylpf = zeros(monte_iter_num);
pitch_max_list_onlylpf = zeros(monte_iter_num);

input_inte_list_onlylpf = zeros(monte_iter_num);
input_max_list_onlylpf = zeros(monte_iter_num);

for con = 1:5
    ld = ld_list(con);
    % max_z0 = max_z0_list(con);

    for loop = 1:monte_iter_num
        if mod(loop,10) == 0
            display(con+":"+loop);
        end
        run("half_model_calculation.m")
        pitch_inte_list_onlylpf(loop) = pitch_integral;
        pitch_max_list_onlylpf(loop) = pitch_max;
        input_inte_list_onlylpf(loop) = input_integral;
        input_max_list_onlylpf(loop) = input_max;
    end
    save("onlylpf/ld="+ld+"-max_z0="+max_z0+".mat","pitch_inte_list_onlylpf","pitch_max_list_onlylpf","input_inte_list_onlylpf","input_max_list_onlylpf");
end
save("total_variables.mat");