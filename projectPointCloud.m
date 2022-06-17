function [img_coords, mask] = projectPointCloud(nuage_points, T_calib, height, width)

img_coords = bsxfun(@rdivide, nuage_points, nuage_points(:, 3));
img_coords = img_coords * T_calib';
img_coords = img_coords(:, 1:2);

mask = img_coords(:, 1) > 0 & img_coords(:, 1) <= width & img_coords(:, 2) > 0 & img_coords(:, 2) <= height;
img_coords = img_coords(mask, :);

end