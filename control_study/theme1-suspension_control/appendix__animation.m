% save_name = "controled";
clear frames;


% first road profile
fig_v = figure('Position', [600 200 1800 480]);
wp = scatter(0,0,1,"white");
hold on;
road=makima(dis,r_p(2,:),TL(1,1)*V-1.4:6.2/500:TL(1,1)*V+10);
road_area = area(TL(1,1)*V-1.4:6.2/500:TL(1,1)*V+10,road,basevalue=-1);
road_profile = plot(TL(1,1)*V-1.4:6.2/500:TL(1,1)*V+10,road,"Color","black",LineWidth=2);
hold on;

% first draw of body
origin_angle = fitsread('animation_materials/ariya.fits',"image");
origin_angle = rescale(origin_angle);
J = imrotate(origin_angle,0,'bilinear','crop');
imwrite(J,'animation_materials/body.png')
img = imread('animation_materials/body.png'); 
y_bias = 225;
x_bias = 345;
img_h = height(img);
img_w = width(img);
resize_ratio = 715;
body_draw = image([-(img_w*(L_f+L_r)/resize_ratio)/2+x_bias*(L_f+L_r)/resize_ratio (img_w*(L_f+L_r)/resize_ratio)/2+x_bias*(L_f+L_r)/resize_ratio],[(img_h*(L_f+L_r)/resize_ratio)/2+y_bias*(L_f+L_r)/resize_ratio -(img_h*(L_f+L_r)/resize_ratio)/2+y_bias*(L_f+L_r)/resize_ratio],img);        % 画像の表示
hold on;

% drawing a circle as a wheel
phi = linspace(0,2*pi,100);
r = 0.7/2;           % 半径
cx_f = TL(1,1)*V+L_f+L_r; cy_f = r+r_p(1,1); % 中心
cx_r = TL(1,1)*V; cy_r = r+r_p(2,1); % 中心

pos_f = [cx_f-r cy_f-r 2*r 2*r];
pos_f2=[cx_f-r+r/3.5 cy_f-r+r/3.5 2*r-2*r/3.5 2*r-2*r/3.5];
circle_as_a_wheel_f = rectangle('Position',pos_f,'Curvature',[1 1],'FaceColor',[0 0 0],'EdgeColor','#000000','LineWidth',2);
circle_as_a_wheel_f2 = rectangle('Position',pos_f2,'Curvature',[1 1],'FaceColor',[.5 .5 .5],'EdgeColor','#000000','LineWidth',2);
% circle_as_a_wheel_f = plot(r*sin(phi)+cx_f,r*cos(phi)+cy_f,Color="black",LineWidth=3);
hold on;
pos_r = [cx_r-r cy_r-r 2*r 2*r];
pos_r2=[cx_r-r+r/3.5 cy_r-r+r/3.5 2*r-2*r/3.5 2*r-2*r/3.5];
circle_as_a_wheel_r = rectangle('Position',pos_r,'Curvature',[1 1],'FaceColor',[0 0 0],'EdgeColor','#000000','LineWidth',2);
circle_as_a_wheel_r2 = rectangle('Position',pos_r2,'Curvature',[1 1],'FaceColor',[.5 .5 .5],'EdgeColor','#000000','LineWidth',2);
% circle_as_a_wheel_r = plot(r*sin(phi)+cx_r,r*cos(phi)+cy_r,Color="black",LineWidth=3);

vector_length_param = 2000;
vec_color_f = [1 0 0];
input_vector_f = quiver(cx_f,cy_f,0,(u(1,1)/vector_length_param),0,"LineWidth",2,Color=vec_color_f);
input_vector_r = quiver(cx_r,cy_r,0,(u(2,1)/vector_length_param),0,"LineWidth",2,Color=vec_color_f);

% time count
txdata = round(TL(1,1),2);
str = {"Time [s]",txdata};
time_text = text(cx_f, 2, str);
time_text.FontSize = 20;
% time_text.FixedWidthFontName("TimesNewRoman")

%% set arounders
grid on;
xl = xlabel('Driving Distance [m]',FontSize=13);
ylabel('Vertical Displacement [m]',FontSize=13);
fontname(fig_v,"Times New Roman")
fontsize(fig_v,16,"points")
axis equal
% ylim([0,3])
ylim([-0.8,2.3])

%% Loop
sampling = true;
s_f = 1;
frames(s_f) = getframe(gcf);
drawnow;
mod_m = 200;
newimg = zeros(371,1140,3);
videoname = "video/"+save_name;
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = (1/dt)/mod_m;
open(video);
for s=2:TL_width/2
    if mod(s,mod_m) == 0
        cx_b = TL(1,s)*V; cy_b = x(1,s);           % displacement
        cx_f = TL(1,s)*V+L_f+L_r; cy_f = r+r_p(1,s); % center of front wheel
        cx_r = TL(1,s)*V; cy_r = r+r_p(2,s);         % center of rear wheel

        road = interp1(dis,r_p(2,:),TL(1,s)*V-1.4:6.2/500:TL(1,s)*V+10,'linear');
        road(isnan(road)) = 0;

        origin_angle = fitsread('animation_materials/ariya.fits',"image");
        origin_angle = rescale(origin_angle);
        J = imrotate(origin_angle,x(2,s)*180/pi,'bilinear','crop');
        imwrite(J,'animation_materials/temp_body.png')

        delete(body_draw);
        delete(circle_as_a_wheel_f);
        delete(circle_as_a_wheel_r);
        delete(circle_as_a_wheel_f2);
        delete(circle_as_a_wheel_r2);
        delete(input_vector_f);
        delete(input_vector_r);
        img = imread('animation_materials/temp_body.png');
        % img(img==0)=255;
        newimg = zeros(371,1140,3);
        newimg(:,:,1)=img(:,:);
        newimg(:,:,2)=img(:,:);
        newimg(:,:,3)=img(:,:);
        newimg = newimg*0.004;
        body_draw = image([-(img_w*(L_f+L_r)/resize_ratio)/2+cx_b+x_bias*(L_f+L_r)/resize_ratio (img_w*(L_f+L_r)/resize_ratio)/2+cx_b+x_bias*(L_f+L_r)/resize_ratio],[(img_h*(L_f+L_r)/resize_ratio)/2+cy_b+y_bias*(L_f+L_r)/resize_ratio -(img_h*(L_f+L_r)/resize_ratio)/2+cy_b+y_bias*(L_f+L_r)/683],newimg);        % 画像の表示
        hold on;
        set(road_profile,'XData',TL(1,s)*V-1.4:6.2/500:TL(1,s)*V+10,"YData",road);
        set(road_area,'XData',TL(1,s)*V-1.4:6.2/500:TL(1,s)*V+10,"YData",road);
        % circle_as_a_wheel_f = plot(r*sin(phi)+cx_f,r*cos(phi)+cy_f,Color="black",LineWidth=3);
        % circle_as_a_wheel_r = plot(r*sin(phi)+cx_r,r*cos(phi)+cy_r,Color="black",LineWidth=3);
        pos_f = [cx_f-r cy_f-r 2*r 2*r];
        pos_f2=[cx_f-r+r/3.5 cy_f-r+r/3.5 2*r-2*r/3.5 2*r-2*r/3.5];
        circle_as_a_wheel_f = rectangle('Position',pos_f,'Curvature',[1 1],'FaceColor',[0 0 0],'EdgeColor','#000000','LineWidth',2);
        circle_as_a_wheel_f2 = rectangle('Position',pos_f2,'Curvature',[1 1],'FaceColor',[.5 .5 .5],'EdgeColor','#000000','LineWidth',2);
        % circle_as_a_wheel_f = plot(r*sin(phi)+cx_f,r*cos(phi)+cy_f,Color="black",LineWidth=3);
        pos_r = [cx_r-r cy_r-r 2*r 2*r];
        pos_r2=[cx_r-r+r/3.5 cy_r-r+r/3.5 2*r-2*r/3.5 2*r-2*r/3.5];
        circle_as_a_wheel_r = rectangle('Position',pos_r,'Curvature',[1 1],'FaceColor',[0 0 0],'EdgeColor','#000000','LineWidth',2);
        circle_as_a_wheel_r2 = rectangle('Position',pos_r2,'Curvature',[1 1],'FaceColor',[.5 .5 .5],'EdgeColor','#000000','LineWidth',2);
        set(wp,'XData',TL(1,s)*V)

        if u(1,s) ~= 0
            vec_color_f = [(1+u(1,s)/abs(u(1,s)))/2 0 (1-u(1,s)/abs(u(1,s)))/2];
        else
            vec_color_f = [1 0 0];
        end
        if u(2,s) ~= 0
            vec_color_r = [(1+u(2,s)/abs(u(2,s)))/2 0 (1-u(2,s)/abs(u(2,s)))/2];
        else
            vec_color_r = [1 0 0];
        end
        input_vector_f = quiver(cx_f,cy_f+r,0,(u(1,s)/vector_length_param),0,"LineWidth",2,Color=vec_color_f);
        input_vector_r = quiver(cx_r,cy_r+r,0,(u(2,s)/vector_length_param),0,"LineWidth",2,Color=vec_color_r);

        txdata = round(TL(1,s),2);
        str = {"Time [s]",txdata};
        time_text.String = str;
        time_text.Position = [cx_f, 2];
        grid on;

        drawnow;
        s_f = s_f + 1;
        frame = getframe(fig_v);
        writeVideo(video,frame);
    end
end
close(video);
% movie(frames,3,(T/dt)/mod_m)