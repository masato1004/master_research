figure_dir = dir(fullfile(scenario,"planning_figure",'*.fig'));

emf_dir = fullfile(figure_dir(1).folder,"planning_emf");
if ~exist(emf_dir,'dir')
    mkdir(emf_dir);
end

strlen = 0;
imax = length(figure_dir);
for i = 1:length(figure_dir)
    openfig(fullfile(figure_dir(i).folder,figure_dir(i).name));
    time = str2double(figure_dir(i).name(1:end-4))-1000;
    for k = 1:3
        subplot(3,1,k);
        fontsize(gca,10,"points");
        % ax = gca; % 現在の軸を取得
        % ax.Position(2) = ax.Position(2) + 0.01; % Y方向に上へずらす
    end
    p = get(gcf,'Position');
    set(gcf,'Position',[p(1) p(2) 255 240]);
    set(gcf, 'Renderer', 'painters');
    print(gcf,fullfile(emf_dir,figure_dir(i).name(1:end-3)+"emf"),'-dmeta','-r600' );
    close(gcf)
    
    Tmp = {'Progress: %3d/%d\n', i, imax};
    Tmp{1} = [ repmat(sprintf('\b'),[1 strlen]),  Tmp{1} ];

    Txt = sprintf(Tmp{1:3});
    fprintf(Txt);
    strlen = length(Txt) - strlen;
end