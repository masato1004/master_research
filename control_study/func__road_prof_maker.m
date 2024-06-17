function [road_total_f,road_total_r,ld,frequency,max_z0,dis_length] = road_prof_maker(shape,TL,T,dt,V,L_f,L_r,dis,start_disturbance,max_z0,ld,const,max_distance,dis_total,dis_total_f,dis_total_r)
    
    % same as paper
    if shape == "_unevenness_"
        Td = ld/V;
        dis_length = 0:max_distance/(T/dt):ld;
        road_total_f = [zeros(1,int32(T*start_disturbance/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(size(dt:max_distance/(T/dt):max_distance-(start_disturbance+ld)))];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(1,width(dis_total)-width([zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld))]))];  % converting rear disturbance and buffer ([m])
        % road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(size(0:max_distance/(T/dt):max_distance-(3+ld)-(L_f+L_r)))];  % converting rear disturbance and buffer ([m])
        
        frequency = 0; max_z0 = max_z0;

    % bump
    elseif shape == "_bump_"
        ld_list = [0.05 0.15 0.05];

        f_dis_total =  [0,start_disturbance,start_disturbance+ld_list(1),start_disturbance+sum(ld_list(1:2)),start_disturbance+sum(ld_list),max_distance];
        r_dis_total =  [0,start_disturbance+L_f+L_r,start_disturbance+L_f+L_r+ld_list(1),start_disturbance+L_f+L_r+sum(ld_list(1:2)),start_disturbance+L_f+L_r+sum(ld_list),max_distance-L_f-L_r];
        road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])
        
        f_dis_total_p = [f_dis_total, dis_total];
        [f_dis_total_p,f_dis_idx] = sort(f_dis_total_p);
        f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);
        road_total_f = f_correct_road_p(ismember(dis_total, f_dis_total_p));

        r_dis_total_p = [r_dis_total, dis_total];
        [r_dis_total_p,r_dis_idx] = sort(r_dis_total_p);
        r_correct_road_p = interp1(r_dis_total,road_total,r_dis_total_p);
        road_total_r = r_correct_road_p(ismember(dis_total, r_dis_total_p));

        frequency = 0; max_z0 = max_z0; dis_length=sum(ld_list);
    

    % sin wave
    elseif shape == "_sin_"
        disturbance_total_f = max_z0*0.5+max_z0*sin(const*dis_total_f-pi/2)/2;        % road disturbance for front wheel ([m])
        disturbance_total_r = max_z0*0.5+max_z0*sin(const*dis_total_r-pi/2)/2;        % road disturbance for rear wheel ([m])
        road_total_f = [zeros(1,int32(T*start_disturbance/(dt*max_distance))), disturbance_total_f];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))+1), disturbance_total_r];  % converting rear disturbance and buffer ([m])
        if width(road_total_f)-width(road_total_r) == 1
            road_total_r=road_total_r(1,1:end-1);
        end

        dis_length=0; ld = 0; max_z0 = max_z0; frequency = (const/(2*pi))*V

    % jari
    elseif shape == "_jari_"
        jari_data = csvread("jari.csv",2,0);
        jari_total = jari_data(:,1);
        road_total = jari_data(:,2)-jari_data(1,2);
        disturbance_total_f = makima(jari_total,road_total,0:max_distance/(T/dt):max_distance-3);
        disturbance_total_r = makima(jari_total,road_total,0:max_distance/(T/dt):max_distance-3-(L_f+L_r));
        road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), disturbance_total_f];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))+1), disturbance_total_r];  % converting rear disturbance and buffer ([m])
        if width(road_total_f)-width(road_total_r) == 1
            road_total_r=road_total_r(1,1:end-1);
        end
        
        frequency = 0; max_z0 = 0; ld = 0; dis_length=0;
    end

end