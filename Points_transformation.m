function warped_nuage_points = Points_transformation(nuage_points, pose)

    warped_nuage_points = bsxfun(@plus, nuage_points * pose(1:3, 1:3)', pose(1:3, 4)');

end