%% font name and size for Automotive Innovation
fontname(gcf,"Times New Roman");
fontsize(gca,11,"points");

%% figure zlim for preview check
zlim([-0.06 0.06])

%% figure size 2d
p = get(gcf,'Position');
set(gcf,'Position',[p(1) p(2) 800 250])

%% figure size 3d
p = get(gcf,'Position');
set(gcf,'Position',[p(1) p(2) 320 260])

%% ptc figure back ground
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

%% labels
xlabel('\itX \rm[m]')
ylabel('\itY \rm[m]')
zlabel('\itZ \rm[m]')