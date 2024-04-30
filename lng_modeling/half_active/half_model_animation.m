close all;
% clc;
clear frames;
warning("off","all")

% figure;
% axis equal
% zb_plt = plot(TL,states(1,1));
% zwf_plt = plot(TL(1,1),states(2,1));
% zwr_plt = plot(TL,states(3,1));
% ang_plt = plot(TL,states(4,1));
% zbdot_plt = plot(TL,states(5,1));
% zwfdot_plt = plot(TL,states(6,1));
% zwrdot_plt = plot(TL,states(7,1));
% angdot_plt = plot(TL,states(8,1));


%% setting for forcusing
% subplot(2,1,1);

% first road profile
fig_v = figure('Position', [600 200 1800 480]);
wp = scatter(0,0,1,"white");
hold on;
if ~sensing
    road=makima(dis_total,road_total_r,states(1,1)-1.4:6.2/500:states(1,1)+10);
else
    road = interp1(road_total_r(:,1)',road_total_r(:,2)',states(1,1)-1.4:6.2/500:states(1,1)+10,'linear');
end
road_area = area(states(1,1)-1.4:6.2/500:states(1,1)+10,road,basevalue=-1);
road_profile = plot(states(1,1)-1.4:6.2/500:states(1,1)+10,road,"Color","black",LineWidth=2);
hold on;

% first draw of body
img = imread('march_body.png'); 
y_bias = 270;
x_bias = 415;
img_h = height(img);
img_w = width(img);
resize_ratio = 792;
body_draw = image([-(img_w*(L_f+L_r)/resize_ratio)/2+x_bias*(L_f+L_r)/resize_ratio (img_w*(L_f+L_r)/resize_ratio)/2+x_bias*(L_f+L_r)/resize_ratio],[(img_h*(L_f+L_r)/resize_ratio)/2+y_bias*(L_f+L_r)/resize_ratio -(img_h*(L_f+L_r)/resize_ratio)/2+y_bias*(L_f+L_r)/resize_ratio],img);        % 画像の表示
hold on;

% drawing a circle as a wheel
phi = linspace(0,2*pi,100);
r = 0.55/2;           % 半径
cx_f = disturbance(1,1)+L_f+L_r; cy_f = r+disturbance(3,1); % 中心
cx_r = disturbance(2,1); cy_r = r+disturbance(4,1); % 中心
circle_as_a_wheel_f = plot(r*sin(phi)+cx_f,r*cos(phi)+cy_f,Color="black",LineWidth=3);
hold on;
circle_as_a_wheel_r = plot(r*sin(phi)+cx_r,r*cos(phi)+cy_r,Color="black",LineWidth=3);

vector_length_param = 2000;
vec_color_f = [1 0 0];
input_vector_f = quiver(cx_f,cy_f,0,(u(1,1)/vector_length_param),0,"LineWidth",2,Color=vec_color_f);
input_vector_r = quiver(cx_r,cy_r,0,(u(2,1)/vector_length_param),0,"LineWidth",2,Color=vec_color_f);

% previewing draw
previewing = fprev_rprev || LQR_fprev_rprev;
if previewing
    camera_pos = scatter(1.88, 1.40+states(1,1), MarkerFaceColor=[0 1 0],MarkerEdgeColor=[0 0 0],SizeData=100);
    fill_x = [1.88 1.88+5.06 1.88+7];
    if sensing
        fill_y = [1.40+states(1,1) interp1(road_total_r(:,1)',road_total_r(:,2)',states(1,1)+5.06,'linear') interp1(road_total_r(:,1)',road_total_r(:,2)',states(1,1)+7,'linear')];
    else
        fill_y = [1.40+states(1,1) makima(TL,road_total_r,states(1,1)+5.06) makima(TL,road_total_r,states(1,1)+7)];
    end
    preview_area = fill(fill_x, fill_y, 'yellow',FaceAlpha=0.2);
end

% time count
txdata = round(TL(1,1),2);
str = {"Time [s]",txdata};
time_text = text(cx_f, 2, str);
time_text.FontSize = 20;
% time_text.FixedWidthFontName("TimesNewRoman")
axis equal

%% set arounders
xl = xlabel('Driving Distance [m]',FontSize=13);
ylabel('Vertical Displacement [m]',FontSize=13);
fontname(fig_v,"Times New Roman")
fontsize(fig_v,16,"points")
axis equal
% ylim([0,3])
ylim([-0.8,2.3])

%% setting for total movement
% road profile
% subplot(2,1,2);
% plot(dis,r_p_f,"Color","black");
% wp_2 = scatter(0,0,1,"white");
% hold on;
% 
% % first draw of body
% body_draw_2 = image([-(1140*3/683)/2+330*3/683 (1140*3/683)/2+330*3/683],[(371*3/683)/2+200*3/683 -(371*3/683)/2+200*3/683],img);        % 画像の表示
% hold on;
% 
% % drawing circles as a wheel% drawing a circle as a wheel
% circle_as_a_wheel_f_2 = plot(r*sin(phi)+cx_f,r*cos(phi)+cy_f,Color="black",LineWidth=3);
% hold on;
% circle_as_a_wheel_r_2 = plot(r*sin(phi)+cx_r,r*cos(phi)+cy_r,Color="black",LineWidth=3);
% axis equal
% xlim([-4,max(dis)])

%% Loop
sampling = true;
cc = 1;
s_f = 1;
frames(s_f) = getframe(gcf);
drawnow;
mod_m = 100;
newimg = zeros(432,1166,3);
videoname = "videos/"+branch+"/"+control+"/"+shape+"/"+figfolder+"-fps-"+(1/dt)/mod_m;
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
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = (1/dt)/mod_m;
open(video);
for s=2:c/2
    if mod(s,mod_m) == 0
        cc = s/(mod_m/10);
%         TL(1,s)
        cx_b = states(1,s); cy_b = states(2,s)-z_b_init;           % displacement
        cx_f = disturbance(1,s)+L_f+L_r; cy_f = r+disturbance(3,s); % center of front wheel
        cx_r = disturbance(2,s); cy_r = r+disturbance(4,s);         % center of rear wheel

        if ~sensing
            road=makima(dis_total,road_total_r,states(1,s)-1.4:6.2/500:states(1,s)+10);
        else
            road = interp1(road_total_r(:,1)',road_total_r(:,2)',states(1,s)-1.4:6.2/500:states(1,s)+10,'linear');
        end

        origin_angle = fitsread('march.fits',"image");
        origin_angle = rescale(origin_angle);
        J = imrotate(origin_angle,states(5,s)*180/pi,'bilinear','crop');
        imwrite(J,'temp_body.png')

        delete(body_draw);
        delete(circle_as_a_wheel_f);
        delete(circle_as_a_wheel_r);
        delete(input_vector_f);
        delete(input_vector_r);
        img = imread('temp_body.png');
        % img(img==0)=255;
        newimg = zeros(img_h,img_w,3);
        newimg(:,:,1)=img(:,:);
        newimg(:,:,2)=img(:,:);
        newimg(:,:,3)=img(:,:);
        newimg = newimg*0.004;
        body_draw = image([-(img_w*(L_f+L_r)/resize_ratio)/2+cx_b+x_bias*(L_f+L_r)/resize_ratio (img_w*(L_f+L_r)/resize_ratio)/2+cx_b+x_bias*(L_f+L_r)/resize_ratio],[(img_h*(L_f+L_r)/resize_ratio)/2+cy_b+y_bias*(L_f+L_r)/resize_ratio -(img_h*(L_f+L_r)/resize_ratio)/2+cy_b+y_bias*(L_f+L_r)/resize_ratio],newimg);        % 画像の表示
        hold on;
        set(road_profile,'XData',states(1,s)-1.4:6.2/500:states(1,s)+10,"YData",road);
        set(road_area,'XData',states(1,s)-1.4:6.2/500:states(1,s)+10,"YData",road);
%         set(body_draw,'XData',[-(1140*(L_f+L_r)/683)/2+cx_b+330*(L_f+L_r)/683 (1140*(L_f+L_r)/683)/2+cx_b+330*(L_f+L_r)/683],'YData',[(371*(L_f+L_r)/683)/2+cy_b+200*(L_f+L_r)/683 -(371*(L_f+L_r)/683)/2+cy_b+200*(L_f+L_r)/683])
%         set(circle_as_a_wheel_f,'XData',r*sin(phi)+cx_f,'YData',r*cos(phi)+cy_f)
        circle_as_a_wheel_f = plot(r*sin(phi)+cx_f,r*cos(phi)+cy_f,Color="black",LineWidth=3);
%         set(circle_as_a_wheel_r,'XData',r*sin(phi)+cx_r,'YData',r*cos(phi)+cy_r)
        circle_as_a_wheel_r = plot(r*sin(phi)+cx_r,r*cos(phi)+cy_r,Color="black",LineWidth=3);
        set(wp,'XData',states(1,s))

        if u(1,cc) ~= 0
            vec_color_f = [(1+u(1,cc)/abs(u(1,cc)))/2 0 (1-u(1,cc)/abs(u(1,cc)))/2];
        else
            vec_color_f = [1 0 0];
        end
        if u(2,cc) ~= 0
            vec_color_r = [(1+u(2,cc)/abs(u(2,cc)))/2 0 (1-u(2,cc)/abs(u(2,cc)))/2];
        else
            vec_color_r = [1 0 0];
        end
%         set(input_vector,'XData',cx_f,'YData',cy_f,'UData',0,'VData',u(1,cc)/vector_length_param,"Color",vec_color);
        input_vector_f = quiver(cx_f,cy_f+r,0,(u(1,cc)/vector_length_param),0,"LineWidth",2,Color=vec_color_f);
        input_vector_r = quiver(cx_r,cy_r+r,0,(u(2,cc)/vector_length_param),0,"LineWidth",2,Color=vec_color_r);

        if previewing
            delete(camera_pos);
            delete(preview_area);
            camera_pos = scatter(cx_r+1.88, 1.40+states(1,s), MarkerFaceColor=[0 1 0],MarkerEdgeColor=[0 0 0],SizeData=100);
            if sampling
                fill_x = [cx_r+1.88 cx_r+1.88+5.06 cx_r+1.88+7];
                if sensing
                    fill_y = [1.40+states(1,s) interp1(road_total_r(:,1)',road_total_r(:,2)',states(1,s)+5.06,'linear') interp1(road_total_r(:,1)',road_total_r(:,2)',states(1,s)+7,'linear')];
                else
                    fill_y = [1.40+states(1,s) 0 0];
                end
                preview_area = fill(fill_x, fill_y, 'yellow',FaceAlpha=0.2);
                sampling = false;
            else
                sampling = true;
            end
        end

        txdata = round(TL(1,s),2);
        str = {"Time [s]",txdata};
        time_text.String = str;
        time_text.Position = [cx_f, 2];

%         g = get(gca);
%         g.Children
%         set(gca, 'Children', [g.Children(3) g.Children(2)])
        drawnow;
        s_f = s_f + 1;
        % frames(s_f) = getframe(fig);
        frame = getframe(fig_v);
        writeVideo(video,frame);
    end
end
close(video);
% movie(frames,3,(T/dt)/mod_m)