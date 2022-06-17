function [pose_rel, error] = get_relative_pose(img_curr, img_prev, dep_prev, T_calib, pyr_levels)

% construct pyramids
img_curr_pyr = construct_pyramid(img_curr, pyr_levels);
img_prev_pyr = construct_pyramid(img_prev, pyr_levels);
dep_prev_pyr = construct_pyramid(dep_prev, pyr_levels);

%% estimate the relative pose from small resolutions to the real image resolution (using pyramid of resolution)

% initialize the relative pose and its increment
pose_rel = eye(4);
increment = zeros(6, 1);

% modify the camera parameters to fit each pyramid resolution
T_calib_pyr = T_calib;
T_calib_pyr(1:2, :) = T_calib_pyr(1:2, :) / (2^(pyr_levels-1));

for n = pyr_levels:-1:1
    
    % image size
    [height, width] = size(dep_prev_pyr{n});
    
    % the 3D coordinates of the pixels of the first image (reference image)
    [nuage_points, mask] = construct_3D_points(dep_prev_pyr{n}, T_calib_pyr);  %the mask is used to eleminate pixels which have a zero depth value

    % the 3D coordinates of the warp points (the estimate pose of the
    % pixels of the second image)
    warped_nuage_points = Points_transformation(nuage_points, pose_rel);
    [warped_img_coordinates, valid_points] = projectPointCloud(warped_nuage_points, T_calib_pyr, height, width);     %valid points is used to represent the points that their projection in the current image has image coordinates which respect the size of the image 
    
    warped_nuage_points = warped_nuage_points(valid_points, :);
    nuage_points = nuage_points(valid_points, :);

    % temporal visual difference (photometric error)
    pht_prev = img_prev_pyr{n};
    pht_prev = pht_prev(mask); % to eleminate pixels which have a zero depth value
    pht_prev = pht_prev(valid_points); % to to eleminate points which have image coordinates out of current image range
    pht_curr = img_curr_pyr{n};
    pht_curr = interp2(pht_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);  %to connect each space point belongs to the image with its own intenisitie value
    pht = pht_curr - pht_prev;  % photometric error

    % Jacobian estimate
    comp_jacobian = compute_Jacobian(warped_nuage_points, nuage_points, pose_rel, T_calib_pyr, img_curr_pyr{n}, warped_img_coordinates);
    
    % increment estimate
    increment = -inv(comp_jacobian'*comp_jacobian)*(comp_jacobian'*pht);

    % get the current relative pose (update the relative pose)
    increment = exprotmT(increment);
    pose_rel = increment * pose_rel;

    % The photometric error of each pyramid level
    [warped_image, mask] = project_points_in_curr_image(img_curr, dep_prev, pose_rel, T_calib);
    error = mean((warped_image(mask) - img_prev(mask)).^2);
    disp(['the photometric error for the level ' num2str(n) ' is ' num2str(error)]);
    
    % increse the focal length
    T_calib_pyr(1:2, :) = T_calib_pyr(1:2, :) * 2;
end
end