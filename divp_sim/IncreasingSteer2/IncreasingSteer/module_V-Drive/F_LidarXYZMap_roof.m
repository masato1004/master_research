function F_LidarXYZMap_roof(lidar_name,clock,HitPoint,SignalStrength,save_dir)
    % global out;

    % x = HitPoint(1,:);
    % y = HitPoint(2,:);
    % z = HitPoint(3,:);
    % c = SignalStrength;
    % s = 4;

    plyfilename = "lidar"+ num2str(clock+1000,'%.3f') + ".pcd";

    % persistent save_dir
    % if isempty(save_dir)
        % save_dir = "pcd_"+lidar_name;
        % listing = dir(save_dir+"/*.png");
        
    if ~exist(save_dir,'dir')
        mkdir(save_dir)
    end
    % end

    % persistent graph;
    % if isempty( graph )
    %     videofilename = lidar_name;
    %     if ispc
    %         filename_mp4 = [videofilename , '.mp4'];
    %         if exist( filename_mp4 ) == 2, delete( filename_mp4 ); end
    %         graph.video = VideoWriter( videofilename, 'MPEG-4' );
    %     elseif isunix
    %         filename_avi = [videofilename , '.avi'];
    %         if exist( filename_avi ) == 2, delete( filename_avi ); end
    %         graph.video = VideoWriter( videofilename );
    %     end
    %     graph.video.FrameRate = 20;
    %     open( graph.video );
    % 
    %     fsize = 20;
    % 
    %     graph.figure = figure("position",[400 100 1000 700]);
    %     graph.plot   = scatter3( x, y, z, s, c, 'filled' );
    %     graph.title  = title( { ...
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
    %         graph.plot.XData    = x;
    %         graph.plot.YData    = y;
    %         graph.plot.ZData    = z;
    %         graph.plot.CData    = c;
    %         graph.plot.SizeData = s;
    %     end
    %     graph.title.String{ 2 } = sprintf( 'Time=%.3f[sec]', clock );
    %     xlim(graph.figure.Children,XRange);
    %     ylim(graph.figure.Children,YRange);
    %     zlim(graph.figure.Children,ZRange);
    %     daspect([1 1 1])
    % 
    %     view(-80,20);
    % 
    % end
    % drawnow();
    % % out.image = zeros( 420, 560, 3, 'uint8' );
    % % out.image(1:420,1:560,:) = imresize(getframe( figure( graph.figure ) ).cdata,[420,560]);
    % out.frame = getframe(graph.figure);
    % writeVideo( graph.video, out.frame );
    pcwrite(pointCloud(HitPoint','Intensity',SignalStrength),save_dir+"/"+plyfilename,'Encoding','ascii');

    % if clock == 25
    %     close(graph.video)
    % end

end
