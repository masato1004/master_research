function [d, k] = F_CalcCrossTrackError(T_ref,x,y)
    [k,~] = dsearchn(T_ref(:,1:2),[x,-y]);
    persistent last_d
    if k ~= 1
        tmp_ref = T_ref([k-1,k+1],1:2);
        [j,~] = dsearchn(tmp_ref,[x,-y]);
        
        a = (T_ref(k,2)-tmp_ref(j,2))/(T_ref(k,1)-tmp_ref(j,1));
        b = T_ref(k,2) - a*T_ref(k,1);
        d = abs(a*x+y+b)/sqrt(a^2+1);
        
        if isnan(d)
            d = last_d;
            disp('d = nan')
        end
    else
        d=0;
    end
    last_d = d;
    
end