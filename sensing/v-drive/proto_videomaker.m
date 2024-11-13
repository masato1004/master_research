load_dir = "image";
listing = dir(load_dir+"/*.png");

if ~exist('video','dir')
    mkdir("video")
end
videoname = "video/test_color";
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = 20;
open(video);

imax = length(listing);
strlen = 0;
for i = 1:imax
    file = imread(load_dir + "/" + listing(i).name);
    writeVideo(video,file);

    Tmp = {'Progress: %3d/%d\n', i, imax};
    Tmp{1} = [ repmat(sprintf('\b'),[1 strlen]),  Tmp{1} ];

    Txt = sprintf(Tmp{1:3});
    fprintf(Txt);
    strlen = length(Txt) - strlen;
end
close(video)