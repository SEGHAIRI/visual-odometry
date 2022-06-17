clear all;
clc;
%% Data_folder
RGB_folder = '/home/stage/Downloads/rgbd_dataset_freiburg1_desk/rgb';
depth_folder =  '/home/stage/Downloads/rgbd_dataset_freiburg1_desk/depth';
groundtruth_folder = '/home/stage/Downloads/rgbd_dataset_freiburg1_desk/groundtruth.txt';

%% Camera intrinsic parameters
T_calib = [481.20 0 319.50;
    0 -480 239.50;
    0 0 1];
%% Initialization
Poses = [];    % Used to store all the poses of the camera
errors = [];   % Used to store all the photometric errors
ground_truth = [];      % Used to store all the ground truth poses

%% ground truth
ground_truth_poses = load(groundtruth_folder);     % Ground truth loading

%% load the pose of the first image frame to use it after that to get the poses of the other frames 
gt_pose_prev = ground_truth_poses(1,2:end);    % quaternion forme [wx, wy, wz, tx, ty, tz]
%gt_pose_prev = [0,0,0,1,0,0,0];    % quaternion forme [wx, wy, wz, tx, ty, tz]
%%  
for im = 2:10
    nom_curr = sprintf('%d.png', im);
    nom_ref = sprintf('%d.png', im-1);
    %% load previous rgb and depth images
    img_prev = im2double(imread(fullfile(RGB_folder, nom_ref)));
    dep_prev = double(imread(fullfile(depth_folder, nom_ref)));

    %% load previous rgb and depth images
    img_curr = im2double(imread(fullfile(RGB_folder, nom_curr)));
    dep_curr = double(imread(fullfile(depth_folder, nom_curr)));

    %% convert the image and depth data
    img_prev = rgb2gray(img_prev); img_curr = rgb2gray(img_curr);
    dep_prev = dep_prev*0.0002; dep_curr = dep_curr*0.0002;   % Multiplying by the scale

    %% load the ground-truth poses of the current image 
    gt_pose_curr = ground_truth_poses(im + 1,2:end);  % quaternion forme [wx, wy, wz, tx, ty, tz]
    %gt_pose_prev = ground_truth_poses(im,2:end);    % quaternion forme [wx, wy, wz, tx, ty, tz]
    %% convert the ground-truth poses from quaternion coordinates to matrix form
    gt_pose_prev = quaternion_to_matrix(gt_pose_prev);
    gt_pose_curr = quaternion_to_matrix(gt_pose_curr);
    
    %% calculate the relative pose between two consecutive frames
    gt_pose_rel = relative_pose(gt_pose_curr, gt_pose_prev);
    
    %% to store the ground_truth poses
    ground_truth = [ground_truth; matrix_to_quaternion(gt_pose_curr)];

    %% calculate the relative pose from two RGB-D frames
    pyr_levels = 7;   % number of pyramid levels
    [pose_rel, error] = get_relative_pose(img_curr, img_prev, dep_prev, T_calib, pyr_levels);
    errors = [errors; error];   %store the photometric errors
    Poses = [Poses ; matrix_to_quaternion(gt_pose_prev * pose_rel)];   %store frame poses
    gt_pose_prev = matrix_to_quaternion(gt_pose_prev * pose_rel);      % update the reference image pose
end