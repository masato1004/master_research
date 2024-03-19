function drawer(x,y,xname,yname,fs,filename)

fig = figure('name',filename,'Position', [500 500 600 190]);
plot(x,y,"LineWidth",2,"Color","#0000ff");
grid on;
% xlim([0,3]);
xlabel(xname);
ylabel(yname);
fontname(fig,"Times New Roman");
fontsize(fig,fs,"points");
if not(exist("figs",'dir'))
    mkdir("figs")
end
saveas(fig,"figs/"+filename);
% saveas(fig,"jpgs/"+filename+".jpg");

end