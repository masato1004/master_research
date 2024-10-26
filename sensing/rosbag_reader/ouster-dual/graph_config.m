%% font name and size for Automotive Innovation
fontname(gcf,"Arial");
fontsize(gca,8,"points");

%% figure zlim for preview check
zlim([-0.06 0.06])

%% figure size
p = get(gcf,'Position');
set(gcf,'Position',[p(1) p(2) 1500 400])