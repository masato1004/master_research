
function du_next = next_input(logi_ctrl,M,F,X,FDW,Fdj,thre,dw_r,dw_prev,dw_fr,pop)
    global num_in num_hid num_out num_w num_b num_nn
    % logi_ctrl = [passive, LQR, rprev, LQR_rprev, fprev_rprev, LQR_fprev_rprev]
    if any(logi_ctrl(3:end-1))
        if any(logi_ctrl(3:4))
            % rprev, LQR_rprev
            for pre=1:M+1
                FDW = FDW + Fdj(:,:,pre)*dw_r(:, (pre));
            end
        else
            % fprev_rprev, LQR_fprev_rprev

            % when use sensor data
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

            % when use actual data
            % for pre=1:M+1
            %     FDW = FDW + Fdj(:,:,pre)*dw_fr(:, pre);
            % end
        end
        if logi_ctrl(4) || logi_ctrl(6)
            % LQR_rprev, LQR_fprev_rprev
            du_next = F*X + FDW;
        else
            % rprev, fprev_rprev
            du_next = FDW;
        end
    elseif logi_ctrl(end-1)
        % LQR
        du_next = F*X;
    else
        
        dzdiff_f = (L_f*X(10)+X(7))-X(8);
        dzdiff_r = (-L_r*X(10)+X(7))-X(9);
        w_IH = reshape(pop(1,1:num_in*num_hid1),[num_hid1,num_in]);
        w_HH = reshape(pop(1,num_in*num_hid1+1:num_w1),[num_hid2,num_hid1]);
        w_HO = reshape(pop(1,num_w1+1:num_w2),[num_out,num_hid2]);
        % b_H1 = reshape(pop(1,num_w2+1:num_w2+num_hid1),[num_hid1,1]);
        % b_H2 = reshape(pop(1,num_w2+num_hid1+1:num_w2+num_hid),[num_hid2,1]);
        % b_O = reshape(pop(1,num_w2+num_hid+1:num_nn),[num_out,1]);

        du_next = purelin(w_HO*(tansig(w_HH*(tansig(w_IH*[X(:);dzdiff_f;dzdiff_r])))));
    end

end