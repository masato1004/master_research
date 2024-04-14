function [road_total_f,road_total_r,ld,frequency,max_z0,dis_length] = road_prof_maker(shape,TL,T,dt,V,L_f,L_r,dis,start_disturbance,max_z0,ld,const,max_distance,dis_total,dis_total_f,dis_total_r)
    
    % sensing
    if shape == "_sensing2_"
        true_datas = load("../line_neo4.mat");
    %     true_datas = load("line_neo4_as_truedata.mat");
        true_profile = true_datas.line_neo4;
        [~,ia,~]=unique(true_profile(:,1));
        true_profile = true_profile(ia,:);
        road_total_f = true_profile;
        road_total_r = true_profile; road_total_r(:,1) = road_total_r(:,1)+(L_f+L_r);
        [~,ia,~]=unique(road_total_r(:,1));
        road_total_r = road_total_r(ia,:,:);

        frequency = 0; max_z0 = 0; ld = 4; dis_length=0;

    % same as paper
    elseif shape == "_paper_"
        Td = ld/V;
        dis_length = 0:max_distance/(T/dt):ld;
        road_total_f = [zeros(1,int32(T*start_disturbance/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(size(dt:max_distance/(T/dt):max_distance-(start_disturbance+ld)))];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(1,width(dis_total)-width([zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld))]))];  % converting rear disturbance and buffer ([m])
        % road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(size(0:max_distance/(T/dt):max_distance-(3+ld)-(L_f+L_r)))];  % converting rear disturbance and buffer ([m])
        
        frequency = 0; max_z0 = max_z0;

    % sin wave
    elseif shape == "_sin_"
        disturbance_total_f = max_z0*0.5+max_z0*sin(const*dis_total_f-pi/2)/2;        % road disturbance for front wheel ([m])
        disturbance_total_r = max_z0*0.5+max_z0*sin(const*dis_total_r-pi/2)/2;        % road disturbance for rear wheel ([m])
        road_total_f = [zeros(1,int32(T*start_disturbance/(dt*max_distance))), disturbance_total_f];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))+1), disturbance_total_r];  % converting rear disturbance and buffer ([m])
        
        ld = 0; max_z0 = max_z0; frequency = (const/(2*pi))*V

    % step
    elseif shape == "_step_"
        road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), max_z0*ones(size(dis_total_f))];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), max_z0*ones(size(dis_total_r))];  % converting rear disturbance and buffer ([m])

        frequency = 0; ld = 0; max_z0 = max_z0;

    % manhole
    elseif shape == "_manhole_"
        manhole_L = 0.6
        m_length = 0:max_distance/(T/dt):manhole_L;
        road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), max_z0*ones(size(m_length)), zeros(size(dt:max_distance/(T/dt):max_distance-(3+manhole_L)))];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), max_z0*ones(size(m_length)), zeros(size(dt:max_distance/(T/dt):max_distance-(3+manhole_L)-(L_f+L_r)))];  % converting rear disturbance and buffer ([m])

        frequency = 0; ld = 0; max_z0 = max_z0;

    % jari
    elseif shape == "_jari_"
        jari_data = csvread("jari.csv",2,0);
        jari_total = jari_data(:,1);
        road_total = jari_data(:,2)-jari_data(1,2);
        disturbance_total_f = makima(jari_total,road_total,0:max_distance/(T/dt):max_distance-3);
        disturbance_total_r = makima(jari_total,road_total,0:max_distance/(T/dt):max_distance-3-(L_f+L_r));
        road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), disturbance_total_f];  % converting front disturbance and buffer ([m])
        road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))+1), disturbance_total_r];  % converting rear disturbance and buffer ([m])

        frequency = 0; max_z0 = 0; ld = 0;
    end

end