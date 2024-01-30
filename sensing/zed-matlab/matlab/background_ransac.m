% RANSAC

function background_ransac(new_ver) 
R = 50;
T = 0.05;
K = 0.2;
[data_num, ~] = size(new_ver);
[a_,b_,c_,d_] = ransac_func(new_ver, R, T, fix(data_num*K));
X_max = max(new_ver(:,1),[],'all');
X_min = min(new_ver(:,1),[],'all');
Y_max = max(new_ver(:,2),[],'all');
Y_min = min(new_ver(:,2),[],'all');
[X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
Z_pre = -a_*X_pre/c_ - b_*Y_pre/c_ - d_/c_;
hold on;
fig = get(groot,'CurrentFigure');
fig.mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");
end