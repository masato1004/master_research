figure_dir = dir(fullfile(scenario,"planning_figure",'*.fig'));

emf_dir = fullfile(figure_dir(1).folder,"planning_emf");
if ~exist(emf_dir,'dir')
    mkdir(emf_dir);
end

strlen = 0;
imax = length(figure_dir);
for i = 1:length(figure_dir)
    openfig(fullfile(figure_dir(i).folder,figure_dir(i).name));
    p = get(gcf,'Position');
    set(gcf,'Position',[p(1) p(2) 320 280]);
    set(gcf, 'Renderer', 'painters');
    print(gcf,fullfile(emf_dir,figure_dir(i).name(1:end-3)+"emf"),'-dmeta','-r600' );
    close(gcf)
    
    Tmp = {'Progress: %3d/%d\n', i, imax};
    Tmp{1} = [ repmat(sprintf('\b'),[1 strlen]),  Tmp{1} ];

    Txt = sprintf(Tmp{1:3});
    fprintf(Txt);
    strlen = length(Txt) - strlen;
end