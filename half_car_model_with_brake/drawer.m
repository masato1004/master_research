function drawer(fig,x,y,i,k)

    % fig = figure('name',i(1),'Position', [500+20*k 500-20*k 600 190]);
    if fig ~= 0
        subplot(2,7,k);
    end
    plot(x,y,"LineWidth",2,"Color","#0000ff");
    grid on;
    % xlim([0,3]);
    xlabel(i(2));
    ylabel(i(3));
    % fontname(fig,"Times New Roman");
    % fontsize(fig,10.5,"points");
    % saveas(fig,"figs/"+i(1));
    % saveas(fig,"jpgs/"+i(1)+".jpg");
    
end