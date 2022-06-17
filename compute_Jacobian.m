function jacobian = calculateWarpingJacobian(warped_nuage_points, nuage_points, pose, T_calib, img_curr, warped_img_coordinates)
%%
fx = T_calib(1, 1);
fy = T_calib(2, 2);

num_points = size(nuage_points, 1);

%%
% spatial gradient in the current frame
[Gx_curr, Gy_curr] = imgradientxy(img_curr, 'CentralDifference');
Gx_curr = interp2(Gx_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);  %to connect each pixel with its own spatial gradient value
Gy_curr = interp2(Gy_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);  %to connect each pixel with its own spatial gradient value
G_curr = cat(2, Gx_curr, Gy_curr);

%%
% projection gradient
% pointcloud after warping
warped_x = warped_nuage_points(:, 1);
warped_y = warped_nuage_points(:, 2);
warped_z = warped_nuage_points(:, 3);

proj_grad = cat(1, ...
    [fx./warped_z, zeros(num_points, 1), -fx.*warped_x./warped_z.^2], ...
    [zeros(num_points, 1), fy./warped_z, -fy.*warped_y./warped_z.^2]); 
proj_grad = reshape(proj_grad, [], 2, 3);

%%
% rigid point-transformation gradient
% pointcloud before warping
x = nuage_points(:, 1);
y = nuage_points(:, 2);
z = nuage_points(:, 3);

rot_grad = kron(eye(3), cat(2, x, y, z));
rot_grad = reshape(rot_grad, [], 3, 9);
trans_grad = kron(eye(3), ones(num_points, 1));
trans_grad = reshape(trans_grad, [], 3, 3);
rigid_trans_grad = cat(3, rot_grad, trans_grad);

%%
% exponential mapping gradient
twist_grad = cat(1, ...
    [0 pose(1, 3) -pose(1, 2) 0 0 0], ...
    [0 pose(2, 3) -pose(2, 2) 0 0 0], ...
    [0 pose(3, 3) -pose(3, 2) 0 0 0], ...
    [-pose(1, 3) 0 pose(1, 1) 0 0 0], ...
    [-pose(2, 3) 0 pose(2, 1) 0 0 0], ...
    [-pose(3, 3) 0 pose(3, 1) 0 0 0], ...
    [pose(1, 2) -pose(1, 1) 0 0 0 0], ...
    [pose(2, 2) -pose(2,1) 0 0 0 0], ...
    [pose(3, 2) -pose(3, 1) 0 0 0 0], ...
    [0 pose(3, 4) -pose(2, 4) 1 0 0;
    -pose(3, 4) 0 pose(1, 4) 0 1 0
    ;pose(2, 4) -pose(1, 4) 0 0 0 1]);

%%
% calculate the chained gradient
chained_gradient = zeros(num_points, 2, 12);
for i = 1:2
    for j = 1:12
        proj_grad_temp = squeeze(proj_grad(:, i, :));
        rigid_trans_grad_temp = squeeze(rigid_trans_grad(:, :, j));
        chained_gradient(:, i, j) = dot(proj_grad_temp, rigid_trans_grad_temp, 2);
    end
end
chained_gradient = reshape(chained_gradient, [], 12) * twist_grad; % multiply with the exponential map jacobian
jacobian = reshape(chained_gradient, [], 2, 6);
jacobian = squeeze(sum(bsxfun(@times, G_curr, jacobian), 2)); % multiply with the spatial gradient => the final jacobian
end