function finish = folder_maker(branch,control,shape,figfolder)
    if not(exist("figs/"+branch,'dir'))
        mkdir("figs/"+branch)
    end
    if not(exist("figs/"+branch+"/"+control,'dir'))
        mkdir("figs/"+branch+"/"+control)
    end
    if not(exist("figs/"+branch+"/"+control+"/"+shape,'dir'))
        mkdir("figs/"+branch+"/"+control+"/"+shape)
    end
    if not(exist("jpgs/"+branch,'dir'))
        mkdir("jpgs/"+branch)
    end
    if not(exist("jpgs/"+branch+"/"+control,'dir'))
        mkdir("jpgs/"+branch+"/"+control)
    end
    if not(exist("jpgs/"+branch+"/"+control+"/"+shape,'dir'))
        mkdir("jpgs/"+branch+"/"+control+"/"+shape)
    end
    if not(exist("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder,'dir'))
        mkdir("figs/"+branch+"/"+control+"/"+shape+"/"+figfolder)
    end
    if not(exist("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder,'dir'))
        mkdir("jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder)
    end
    finish = true;
end