function F_Video()
frame = getframe(gcf);
global video
if isempty(video)
    videoname = 'test_tracking';
    video = VideoWriter(videoname,'MPEG-4');
    video.FrameRate = 10;
    open(video);
end

writeVideo(video,frame);
end