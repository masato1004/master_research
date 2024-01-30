h = 1080;
w = 1920;
X = disparityMap*10;
[X, Y] = meshgrid(w/w:w/w:w,h/h:h/h:h);

figure;
S = 3;
X_max = max(X,[],"all");
C = ones(h,w,3);
cl = X/X_max;
for r = 1:h
    for c = 1:w
        C(r,c,:) = cl(r,c);
    end
end
C = reshape(C,h*w,1,3);
C = reshape(C,h*w,3);
data = cat(2,reshape(X_max-X,h*w,1),reshape(w-Y,h*w,1),reshape(h-Z,h*w,1));
[a_,b_,c_,d_] = ransac_func(data);

% draw predicted surface+
[X_pre, Y_pre] = meshgrid(0:X_max/8:X_max,0:w/8:w);
Z_pre = -a_*X_pre/c_ - b_*Y_pre/c_ - d_/c_;

mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");
hold on;
scatter3(reshape(X_max-X,h*w,1),reshape(w-Y,h*w,1),reshape(h-Z,h*w,1),S,C,"AlphaVariable",0.3);