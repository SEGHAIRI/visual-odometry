function [warped_image, mask] = project_points_in_curr_image(img_curr, dep_prev, pose_rel, T_calib)

[height, width] = size(img_curr);

[nuage_points, mask] = construct_3D_points(dep_prev, T_calib);
warped_nuage_points = Points_transformation(nuage_points, pose_rel);
[warped_img_coords, warped_mask] = projectPointCloud(warped_nuage_points, T_calib, height, width);

warped_intensities = interp2(img_curr, warped_img_coords(:, 1), warped_img_coords(:, 2), 'linear', 0);

valid_indices = find(mask);
valid_indices = valid_indices(warped_mask);
mask = zeros(height, width);
mask(valid_indices) = 1;
mask = logical(mask);

warped_image = zeros(height, width);
warped_image(valid_indices) = warped_intensities;

end