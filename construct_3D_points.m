function [nuage_points, mask] = construct_3D_points(depth, T_calib)

    [height,width] = size(depth);
    
    fx = T_calib(1, 1);
    fy = T_calib(2, 2);
    u0 = T_calib(1, 3);
    v0 = T_calib(2, 3);
    
    [x, y] = meshgrid(1:width, 1:height);
    x = (x - u0)/fx;
    y = (y - v0)/fy;
    
    normalized_coords = cat(3, x, y);
    
    mask = depth > 0;   %binary mask used to eliminate wrong depth values(values equal to zero) 
    
    nuage_points = cat(3, bsxfun(@times, normalized_coords, depth), depth);

    % convert pixel coordinnates to a list
    nuage_points = reshape(nuage_points, [], 3);
    nuage_points = nuage_points(mask, :);

end