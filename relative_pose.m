function pose_rel = relative_pose(pose_curr, pose_prev)

pose_rel = T_inv(pose_prev) * pose_curr;

end