%% Matrix equation of motion
function dx = motion_func(X,U)
global m_b, m_w, k_sf, k_sr, k_w, c_sf, c_sr, c_w, I_b, L_f, L_r

M_mat = ;

C_mat = ;
K_mat = ;