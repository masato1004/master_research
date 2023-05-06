function du_next = next_input(logi_ctrl,M,F,X,FDW,Fdj,thre,dw_r,dw_prev,dw_fr)
    
    % logi_ctrl = [passive, LQR, rprev, LQR_rprev, fprev_rprev, LQR_fprev_rprev]
    if logi_ctrl(1)
        du_next = zeros(2,1);
    elseif any(logi_ctrl(3:end))
        if any(logi_ctrl(3:4))
            % rprev, LQR_rprev
            for pre=1:M+1
                FDW = FDW + Fdj(:,:,pre)*dw_r(:, (pre));
            end
        else
            % fprev_rprev, LQR_fprev_rprev
            if thre <= 0
                for pre=1:M+1
                    FDW = FDW + Fdj(:,:,pre)*dw_prev(:, pre);
                    % FDW = FDW + Fdj(:,:,pre)*dw_fr(:, (pre-1));
                end
            else
                for pre=1:M+1
                    FDW = FDW + Fdj(:,:,pre)*dw_r(:, (pre));
                end
            end
        end
        if logi_ctrl(4) || logi_ctrl(6)
            % LQR_rprev, LQR_fprev_rprev
            du_next = F*X + FDW;
        else
            % rprev, fprev_rprev
            du_next = FDW;
        end
    else
        % LQR
        du_next = F*X;
    end
end