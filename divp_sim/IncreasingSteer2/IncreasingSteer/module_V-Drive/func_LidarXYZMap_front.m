function func_LidarXYZMap_front(lidar_name,clock,HitPoint,SignalStrength,save_dir)
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

    % persistent graph_2;
    % if isempty( graph_2 )
    %     videofilename = lidar_name;
    %     if ispc
    %         filename_mp4 = [videofilename , '.mp4'];
    %         if exist( filename_mp4 ) == 2, delete( filename_mp4 ); end
    %         graph_2.video = VideoWriter( videofilename, 'MPEG-4' );
    %     elseif isunix
    %         filename_avi = [videofilename , '.avi'];
    %         if exist( filename_avi ) == 2, delete( filename_avi ); end
    %         graph_2.video = VideoWriter( videofilename );
    %     end
    %     graph_2.video.FrameRate = 20;
    %     open( graph_2.video );
    % 
    %     fsize = 20;
    % 
    %     graph_2.figure = figure("position",[400 100 1000 700]);
    %     graph_2.plot   = scatter3( x, y, z, s, c, 'filled' );
    %     graph_2.title  = title( { ...
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
    %         graph_2.plot.XData    = x;
    %         graph_2.plot.YData    = y;
    %         graph_2.plot.ZData    = z;
    %         graph_2.plot.CData    = c;
    %         graph_2.plot.SizeData = s;
    %     end
    %     graph_2.title.String{ 2 } = sprintf( 'Time=%.3f[sec]', clock );
    %     xlim(graph_2.figure.Children,XRange);
    %     ylim(graph_2.figure.Children,YRange);
    %     zlim(graph_2.figure.Children,ZRange);
    %     daspect([1 1 1])
    % 
    %     view(-80,20);
    % 
    % end
    % drawnow();
    % % out.image = zeros( 420, 560, 3, 'uint8' );
    % % out.image(1:420,1:560,:) = imresize(getframe( figure( graph.figure ) ).cdata,[420,560]);
    % out.frame2 = getframe(graph_2.figure);
    % writeVideo( graph_2.video, out.frame2 );
    pcwrite(pointCloud(HitPoint','Intensity',SignalStrength),save_dir+"/"+plyfilename,'Encoding','ascii');

    % if clock == 25
    %     close(graph_2.video)
    % end
end
