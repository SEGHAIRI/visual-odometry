function pose_mat = quaternion_to_matrix(pose_vec)

trans_vec = pose_vec(1:3);

quaternion = [pose_vec(end), pose_vec(4:end-1)];
rot_mat = quat2rotm(quaternion);

pose_mat = eye(4);
pose_mat(1:3, 1:3) = rot_mat;
pose_mat(1:3, 4) = trans_vec;

end