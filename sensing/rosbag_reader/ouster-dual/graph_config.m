%% font name and size for Automotive Innovation
fontname(gcf,"Arial");
fontsize(gca,13,"points");

%% figure zlim for preview check
zlim([-0.06 0.06])

%% figure size 2d
p = get(gcf,'Position');
set(gcf,'Position',[p(1) p(2) 150 130])

%% figure size 3d
p = get(gcf,'Position');
set(gcf,'Position',[p(1) p(2) 500,340])