function depth_for_img = func_projectLidarToDepthImage(cameraPoints, fl, pp, ims, rd, td)
% % LiDAR点群をホモジニアス座標に変換
% lidarPoints_h = [lidarPoints, ones(size(lidarPoints, 1), 1)];  % Nx4

% % LiDAR点をカメラ座標系に変換
% cameraPoints_h = (T_lidar_to_camera * lidarPoints_h')';       % Nx4
% cameraPoints = cameraPoints_h(:, 1:3);                        % Nx3

% カメラ視野内（Z > 0）の点をフィルタリング
validIdx = cameraPoints(:, 3) > 2;
cameraPoints = cameraPoints(validIdx, :);

% 正規化画像平面に投影
normP = cameraPoints(:, 1:2) ./ cameraPoints(:, 3);

% ラジアルおよびタンジェンシャル歪みの適用
r2 = sum(normP.^2, 2);
radFact = (1+rd(1)*r2+rd(2)*r2.^2+rd(3)*r2.^3) / (1+rd(4)*r2+rd(5)*r2.^2+rd(6)*r2.^3);

xDistorted = normP(:, 1) .* radFact + 2 * td(1) * normP(:, 1) .* normP(:, 2) + td(2) * (r2 + 2 * normP(:, 1).^2);
yDistorted = normP(:, 2) .* radFact + td(1) * (r2 + 2 * normP(:, 2).^2) + 2 * td(2) * normP(:, 1) .* normP(:, 2);

% ピクセル座標に変換
imagePoints = [fl(1) * xDistorted + pp(1), fl(2) * yDistorted + pp(2)];

% 画像サイズ内の点をフィルタリング
inBoundsIdx = (imagePoints(:,1)>=1) & (imagePoints(:,1)<=ims(2)) & (imagePoints(:,2)>=1) & (imagePoints(:,2)<=ims(1)) & ~isnan(imagePoints(:, 1)) & ~isnan(imagePoints(:, 2));
% inBoundsIdx = (imagePoints(:, 1) >= -int(ims(2)/2) & imagePoints(:, 1) < int(ims(2)/2) & imagePoints(:, 2) >= -int(ims(1)/2) & imagePoints(:, 2) < int(ims(1)/2));
imagePoints = imagePoints(inBoundsIdx, :);
depths = cameraPoints(inBoundsIdx, 3);  % 各点の深度（カメラ座標系のZ値）

depth_for_img = [imagePoints,depths];
disp('done')

% % 深度画像の作成
% depthImage = nan(ims);  % NaNで初期化（表示範囲外の部分）
% for i = 1:size(imagePoints, 1)
%     x = round(imagePoints(i, 1));
%     y = round(imagePoints(i, 2));
%     depthImage(y, x) = depths(i);
% end
% 
% % 深度画像の表示
% figure;
% imagesc(depthImage);
% colorbar;
% colormap('jet');
% title('Depth Image from LiDAR Points');
% xlabel('X (pixels)');
% ylabel('Y (pixels)');
end