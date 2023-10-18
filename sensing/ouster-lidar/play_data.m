player = pcplayer([-5 5],[-5 5],[-5 5]);

listing = dir(savedir+"/*.ply");
k = 1;

while isOpen(player) 
    ptCloud = pcread(listing(k));
    view(player,ptCloud);
    k = k + 1;
    if k > length(listing)
        break
    end
end