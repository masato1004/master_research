function conditions = folder_maker(branch,control,shape,figfolder,smoothing_method,added_noise)
    % figs
    if not(exist("figs/"+branch,'dir'))
        mkdir("figs/"+branch)
    end
    if not(exist("figs/"+branch+"/"+control,'dir'))
        mkdir("figs/"+branch+"/"+control)
    end
    if not(exist("figs/"+branch+"/"+control+"/"+shape,'dir'))
        mkdir("figs/"+branch+"/"+control+"/"+shape)
    end
    if not(exist("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder,'dir'))
        mkdir("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder)
    end
    if not(exist("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method,'dir'))
        mkdir("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method)
    end
    if not(exist("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method+"--"+added_noise,'dir'))
        mkdir("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method+"--"+added_noise)
    end

    % jpgs
    if not(exist("jpgs/"+branch,'dir'))
        mkdir("jpgs/"+branch)
    end
    if not(exist("jpgs/"+branch+"/"+control,'dir'))
        mkdir("jpgs/"+branch+"/"+control)
    end
    if not(exist("jpgs/"+branch+"/"+control+"/"+shape,'dir'))
        mkdir("jpgs/"+branch+"/"+control+"/"+shape)
    end
    if not(exist("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder,'dir'))
        mkdir("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder)
    end
    if not(exist("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method,'dir'))
        mkdir("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method)
    end
    if not(exist("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method+"--"+added_noise,'dir'))
        mkdir("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method+"--"+added_noise)
    end

    % video
    if not(exist("videos",'dir'))
        mkdir("videos");
    end
    if not(exist("videos/"+branch,'dir'))
        mkdir("videos/"+branch);
    end
    if not(exist("videos/"+branch+"/"+control,'dir'))
        mkdir("videos/"+branch+"/"+control);
    end
    if not(exist("videos/"+branch+"/"+control+"/"+shape,'dir'))
        mkdir("videos/"+branch+"/"+control+"/"+shape);
    end
    if not(exist("videos/"+branch+"/"+control+"/"+shape+"/"+figfolder,'dir'))
        mkdir("videos/"+branch+"/"+control+"/"+shape+"/"+figfolder)
    end
    if not(exist("videos/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method,'dir'))
        mkdir("videos/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method)
    end
    if not(exist("videos/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method+"--"+added_noise,'dir'))
        mkdir("videos/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method+"--"+added_noise)
    end

    % conditions name
    conditions = branch+"/"+control+"/"+shape+"/"+figfolder+"/"+smoothing_method+"--"+added_noise;
end