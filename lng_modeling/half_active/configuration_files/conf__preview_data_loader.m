% LOADING
prev_start = 5.06;
prev_end = 7;
current_dis = r_p_prev(1,1);
if sensing
    load_dir = "../preview_datas";
    listing = dir(load_dir+"/*.mat");
    file = load(load_dir + "/" + listing(2).name);
    vertices = file.vertices;
    
    wf_local = previewing(vertices);
    data_end = wf_local(2,round(end/2));
    wf_global = wf_local; last_minimum = 1.9;
else
    listing = [];
    wf_local = [
        prev_start:tc*V:prev_end;
        makima(r_p_prev(1,:),zeros(size(r_p_prev(1,:))),current_dis+prev_start:tc*V:current_dis+prev_end)
        ];
    wf_global = wf_local; last_minimum = wf_global(1,1);
end
wf_grad = [
    wf_global(1,:)./V;
    wf_global(2,:);
    gradient(wf_global(2,:))./(gradient(wf_global(1,:))./V)
    ];
dw_prev = zeros(4,Md+1);