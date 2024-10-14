function folder_maker(dir_name)
    % figs
    if not(exist(dir_name,'dir'))
        mkdir(dir_name)
    end
end