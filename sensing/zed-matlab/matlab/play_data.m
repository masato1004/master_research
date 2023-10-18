player = pcplayer([-5 5],[-5 5],[-5 5]);

listing = dir(savedir_lidar+"/*.ply");
k = 1;

while isOpen(player) 
    ptCloud = pcread(savedir_lidar+listing(k).name);
    view(player,ptCloud);
    k = k + 1;
    if k > length(listing)
        break
    end
end