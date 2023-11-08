% function F = background_pointcloud(vertices, F, thr)
% 
%     [row_size, ~] = size(vertices);
%     
%     if row_size ~= 0
%         if row_size > thr
%             ptc_ver = pointCloud(vertices(end-thr-1:end, :));
%         else
%             ptc_ver = pointCloud(vertices);
%         end
% %         pcshow(ptc_ver);
% %         drawnow;
%         F = [F;ptc_ver];
%     end
% 
% 
% end
%% makeVideo

list = dir('ZEDmat');
len = length(list);
fname = list(len)
fname = strcat('ZEDmat/',fname.name);
load(fname);
fname = strrep(fname,'.mat','.avi');
fname = strrep(fname,'mat','video');

[rs, ~] = size(F);
v = VideoWriter(fname);
open(v);
for r = 1:rs
    r
    pcshow(F(r));
    view([-1 5 -1.5]);
    xlabel('traveling Direction','FontSize',12);
    zlabel('Height','FontSize',12);
    drawnow;
%     frames(r) = getframe(gcf);
    f = getframe(gcf);
    writeVideo(v,f);
end
% movie(frames)
close(v);