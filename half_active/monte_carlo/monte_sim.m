monte_iter_num = 1000;

ld_list = [0.5, 1, 2, 2.5];
max_z0_list = [0.01, 0.045, 0.08];


pitch_inte_list_nonfilter = zeros(1,monte_iter_num);
pitch_max_list_nonfilter = zeros(1,monte_iter_num);

input_inte_list_nonfilter = zeros(1,monte_iter_num);
input_max_list_nonfilter = zeros(1,monte_iter_num);


pitch_inte_list_onlylpf = zeros(1,monte_iter_num);
pitch_max_list_onlylpf = zeros(1,monte_iter_num);

input_inte_list_onlylpf = zeros(1,monte_iter_num);
input_max_list_onlylpf = zeros(1,monte_iter_num);


pitch_inte_list = zeros(1,monte_iter_num);
pitch_max_list = zeros(1,monte_iter_num);

input_inte_list = zeros(1,monte_iter_num);
input_max_list = zeros(1,monte_iter_num);

wf_global_nonfilter = {};
wf_global_onlylpf = {};
wf_global = {};

for conz = 1:width(max_z0_list)
    for conl = 1:width(ld_list)
        ld = ld_list(conl);
        max_z0 = max_z0_list(conz);

        for loop = 1:monte_iter_num
            if mod(loop,10) == 0
                display(con+":"+loop);
            end
            wa  = false;
            lpf = false;
            run("half_model_calculation.m")
            pitch_inte_list_nonfilter(1,loop) = pitch_integral;
            pitch_max_list_nonfilter(1,loop) = pitch_max;
            input_inte_list_nonfilter(1,loop) = input_integral;
            input_max_list_nonfilter(1,loop) = input_max;
            wf_global_nonfilter(loop) = {[wf_global(1,:)+T*V; wf_global(2,:)]};
            clear wf_global;
            
            wa  = false;
            lpf = true;
            run("half_model_calculation.m")
            pitch_inte_list_onlylpf(1,loop) = pitch_integral;
            pitch_max_list_onlylpf(1,loop) = pitch_max;
            input_inte_list_onlylpf(1,loop) = input_integral;
            input_max_list_onlylpf(1,loop) = input_max;
            wf_global_onlylpf(loop) = {[wf_global(1,:)+T*V; wf_global(2,:)]};
            clear wf_global;

            wa  = true;
            lpf = true;
            run("half_model_calculation.m")
            pitch_inte_list(1,loop) = pitch_integral;
            pitch_max_list(1,loop) = pitch_max;
            input_inte_list(1,loop) = input_integral;
            input_max_list(1,loop) = input_max;
            wf_global(loop) = {[wf_global(1,:)+T*V; wf_global(2,:)]};
            clear wf_global;
        end
        save("nonfiltered_specify/ld="+ld+"-max_z0="+max_z0+".mat","pitch_inte_list_nonfilter","pitch_max_list_nonfilter","input_inte_list_nonfilter","input_max_list_nonfilter","wf_global_nonfilter","r_p_prev");
        save("onlylpf_specify/ld="+ld+"-max_z0="+max_z0+".mat","pitch_inte_list_onlylpf","pitch_max_list_onlylpf","input_inte_list_onlylpf","input_max_list_onlylpf","wf_global_onlylpf","r_p_prev");
        save("filtered_specify/ld="+ld+"-max_z0="+max_z0+".mat","pitch_inte_list","pitch_max_list","input_inte_list","input_max_list","wf_global","r_p_prev");
    end
end
save("total_variables.mat");