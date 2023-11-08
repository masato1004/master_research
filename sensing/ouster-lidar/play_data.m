% player = pcplayer([-4 4],[-2 6],[-4 4]);
fig = figure('name',"point cloud"); grid on;

listing = dir(savedir_lidar+"*.ply");
k = 1;

videoname = 'video';
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = 20;
open(video);
% while isOpen(player) 
while k <= length(listing)
    ptCloud = pcread(savedir_lidar+listing(k).name);
    % view(player,ptCloud);
    pcshow(ptCloud)
    xlim([-3,1])
    ylim([-1,10])
    zlim([-2,2.5])
    
    xlabel('\itX \rm[m]')
    ylabel('\itY \rm[m]')
    zlabel('\itZ \rm[m]')
    fontname(fig,"Times New Roman");
    fontsize(fig,12,"points");
    drawnow;
    k = k + 1;

    frame = getframe(fig);
    writeVideo(video,frame);

    if k > length(listing)
        break
    end
end
close(video);