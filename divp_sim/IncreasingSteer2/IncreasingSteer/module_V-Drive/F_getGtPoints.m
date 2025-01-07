function F_getGtPoints(points_name,clock,Points,pos,roll,pitch,yaw)
    % global out;

    points = reshape(Points,[height(Points)*width(Points), 3]);

    try
        gridStep = 0.015;
        ptCloud = pcdownsample(pointCloud(points),'gridAverage',gridStep);
    catch
        ptCloud = pointCloud(points);
    end

    % x = points(:,1)';
    % y = points(:,2)';
    % z = points(:,3)';
    % c = SignalStrength;   
    % s = 4;

    pcdfilename = "gt_points"+ num2str(clock+1000,'%.3f') + ".pcd";

    persistent save_dir camera_position
    if isempty(save_dir)
        save_dir = "pcd_"+points_name;

        camera_position = [1.881159, 0.0, 1.554000];
        % listing = dir(save_dir+"/*.png");
        
        if ~exist(save_dir,'dir')
            mkdir(save_dir)
        end
    end

    % persistent map_points
    % agl = [yaw(1,1),pitch(1,1),roll(1,1)];
    % pos = pos(:,1)';
    % 
    % idx = points(:,3)<20&points(:,1)<20&points(:,1)>0&points(:,2)<5&points(:,2)>-5;
    % ptCloud = [points(idx,1),-points(idx,2),points(idx,3)];
    % bonnet_idx = ptCloud(:,1)<3&ptCloud(:,2)<2.5&ptCloud(:,2)>-2.5&ptCloud(:,3)>-1.2;
    % ptCloud = pointCloud(ptCloud(~bonnet_idx,:));
    % 
    % R1 = eul2rotm(agl);
    % A1 = [[R1;0,0,0],[pos';1]];
    % ptCloud = pctransform(pointCloud([ptCloud.Location(:,1)+camera_position(1),ptCloud.Location(:,2),ptCloud.Location(:,3)+camera_position(3)]),rigidtform3d(A1));

    % if isempty(map_points)
    %     map_points=ptCloud.Location;
    % else
    %     map_points = [map_points; ptCloud.Location];
    % end


    % persistent gt_point_graph;
    % if isempty( gt_point_graph )
    %     videofilename = points_name;
    %     if ispc
    %         filename_mp4 = [videofilename , '.mp4'];
    %         if exist( filename_mp4 ) == 2, delete( filename_mp4 ); end
    %         gt_point_graph.video = VideoWriter( videofilename, 'MPEG-4' );
    %     elseif isunix
    %         filename_avi = [videofilename , '.avi'];
    %         if exist( filename_avi ) == 2, delete( filename_avi ); end
    %         gt_point_graph.video = VideoWriter( videofilename );
    %     end
    %     gt_point_graph.video.FrameRate = 20;
    %     open( gt_point_graph.video );
    % 
    %     fsize = 20;
    % 
    %     gt_point_graph.figure = figure("position",[400 100 1000 700]);
    %     gt_point_graph.plot   = scatter3( x, y, z, s, z, 'filled' );
    %     gt_point_graph.title  = title( { ...
    %         'LiDAR'; ...
    %         'Time=0[sec]' }, 'FontSize', fsize, 'Fontname', 'Arial');
    %     view(-80,20); box on; grid on; daspect( [ 1, 1, 1 ] ); colormap( 'jet' );
    %     xlabel( 'X [m]', 'FontSize', fsize, 'Fontname', 'Arial' );
    %     ylabel( 'Y [m]', 'FontSize', fsize, 'Fontname', 'Arial' );
    %     zlabel( 'Z [m]', 'FontSize', fsize, 'Fontname', 'Arial' );
    % else
    %     XRange = [-20 40];
    %     YRange = [-20 20];
    %     ZRange = [-5 10];
    % 
    %     figure(1);
    %     % scatter3(x,y,z,s,c,'filled');
    % 
    %     if numel( x ) > 0
    %         gt_point_graph.plot.XData    = x;
    %         gt_point_graph.plot.YData    = y;
    %         gt_point_graph.plot.ZData    = z;
    %         gt_point_graph.plot.CData    = z;
    %         gt_point_graph.plot.SizeData = s;
    %     end
    %     gt_point_graph.title.String{ 2 } = sprintf( 'Time=%.3f[sec]', clock );
    %     xlim(gt_point_graph.figure.Children,XRange);
    %     ylim(gt_point_graph.figure.Children,YRange);
    %     zlim(gt_point_graph.figure.Children,ZRange);
    %     daspect([1 1 1])
    % 
    %     view(-80,20);
    % 
    % end
    % drawnow();
    % out.image = zeros( 420, 560, 3, 'uint8' );
    % out.image(1:420,1:560,:) = imresize(getframe( figure( gt_point_graph.figure ) ).cdata,[420,560]);
    % out.frame = getframe(gt_point_graph.figure);
    % writeVideo( gt_point_graph.video, out.frame );
    if ~isempty(points)
        pcwrite(ptCloud,save_dir+"/"+pcdfilename,'Encoding','ascii');
        % pcwrite(pointCloud(map_points),save_dir+"/map_gt",'Encoding','ascii');
    end

    % if clock == 25
    %     close(gt_point_graph.video)
    % end
end