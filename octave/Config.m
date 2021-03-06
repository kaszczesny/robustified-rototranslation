function conf = Config()
% return config object


conf=struct();

conf.cheat = 0; %whether to use rgbd and groundtruth for calculations

%output images settings
conf.save_images = 1;
conf.output_folder = strftime("../results/%y-%m-%d_%H-%M-%S", localtime(time()));

% main() setting

conf.frame_start = 1000;
conf.frame_interval = 2;
conf.n_frames = 100;

conf.TUM = 1;

%KITTI
%{
persistent files = glob("../../00/image_0/*.png");
%persistent files = glob("../../01/image_0/*.png");
%persistent files = glob("../../02/image_0/*.png");
conf.im_name = @(x) files{x};
conf.im_name_depth = @(x) 0;
conf.TUM = 0;
%}
%TUM
%
%persistent files = glob("../../rgbd_dataset_freiburg3_long_office_household/rgb/*.png");
persistent files = glob("../../rgbd_dataset_freiburg3_teddy/rgb/*.png");
conf.im_name = @(x) files{x};
%persistent files_depth = glob("../../rgbd_dataset_freiburg3_long_office_household/depth/*.png");
persistent files_depth = glob("../../rgbd_dataset_freiburg3_teddy/depth/*.png");
files_depth_mapping = csvread("../data/TUM/depth_idx.txt");
conf.im_name_depth = @(x) files_depth{files_depth_mapping(x)};
%
%{
%persistent files = glob("../../rgbd_dataset_freiburg2_desk_with_person/rgb/*.png");
%persistent files = glob("../../rgbd_dataset_freiburg2_pioneer_slam2/rgb/*.png");
persistent files = glob("../../rgbd_dataset_freiburg2_desk/rgb/*.png");
%persistent files = glob("../../rgbd_dataset_freiburg2_360_kidnap/rgb/*.png");
conf.im_name = @(x) files{x};
%persistent files_depth = glob("../../rgbd_dataset_freiburg2_pioneer_slam2/depth/*.png");
persistent files_depth = glob("../../rgbd_dataset_freiburg2_desk/depth/*.png");
%persistent files_depth = glob("../../rgbd_dataset_freiburg2_desk_with_person/depth/*.png");
%persistent files_depth = glob("../../rgbd_dataset_freiburg2_360_kidnap/depth/*.png");
files_depth_mapping = csvread("../data/TUM/depth_idx.txt");
conf.im_name_depth = @(x) files_depth{files_depth_mapping(x)};
%}
%{
persistent files = glob("../../rgbd_dataset_freiburg1_xyz/rgb/*.png");
%persistent files = glob("../../rgbd_dataset_freiburg1_plant/rgb/*.png");
conf.im_name = @(x) files{x};
persistent files_depth = glob("../../rgbd_dataset_freiburg1_xyz/depth/*.png");
%persistent files_depth = glob("../../rgbd_dataset_freiburg1_plant/depth/*.png");
files_depth_mapping = csvread("../data/TUM/depth_idx.txt");
conf.im_name_depth = @(x) files_depth{files_depth_mapping(x)};
%}

%%%%%%%%%%%%%%%%%%%%%

conf.scale = 1;
persistent imgsize = size(imresize(imread(conf.im_name(1)), conf.scale))(1:2);
conf.imgsize = imgsize(end:-1:1);

%%%%%%%%%%%%%%%%%%%%%

%KITTI
%{
conf.principal_point =  [607.1928 185.2157] * conf.scale; %sort of half, xyz
conf.zf = 718.856 * conf.scale; %todo: zf_x and zf_y?
conf.FPS = 9.65; % todo: get actual value from dataset
%}
%TUM fr3
%
conf.principal_point =  [320.1 247.6] * conf.scale; %sort of half, xyz
conf.zf = mean([535.4, 539.2]) * conf.scale; %X
conf.FPS = 9.65; % todo: get actual value from dataset
%
%TUM fr2
%{
conf.principal_point =  [325.1 249.7] * conf.scale; %sort of half, xyz
conf.zf = mean([520.9, 521.0]) * conf.scale;
conf.FPS = 9.65; % todo: get actual value from dataset
%}
%TUM fr1
%{
conf.principal_point =  [318.6 255.3] * conf.scale; %sort of half, xyz
conf.zf = mean([517.3, 516.5]) * conf.scale;
conf.FPS = 9.65; % todo: get actual value from dataset
%}


  % DoG:
conf.ksize = 13;
%conf.sigma1 = 3; 
%conf.sigma2 = 2;
conf.sigma1 = 1.7818;
conf.sigma2 = 2.30029;

  % Test 1:
conf.THRESH_GRAD_MIN = 0.03;
conf.THRESH_GRAD_MAX = 0.03;
conf.THRESH_GRAD_GAIN = 1e-6;
conf.KL_REFERENCE_NUMBER = 5000;
conf.max_img_value = 255;
conf.detector_dog_thresh = 0.095259868922420; %Relation between DoG threshold and Gradient threshold ~1/Sigma0^4

  % Test 2:
conf.win_s = 2;
conf.PosNegThresh = 0.2;

%figure 1, 2, 3, 4
conf.visualize_edges = 0;
%figure 5
conf.visualize_crossing = 0;
%figure 6, 7, 8, 40, 41
conf.visualize_minimizer_insides = 0;
%figure 9
conf.visualize_score = 0;
%figure 10, 11, 12, 13
conf.visualize_matches = 0;
conf.visualize_matches_step = 100;
%figure 14, 25
conf.visualize_depth = 1;
%figure 15
conf.visualize_history = 0;
%figure 16, 20, 21, 22, 23, 24
conf.visualize_regularization = 0;
%figure 17, 26, 27, 28, 29, 30
conf.visualize_dmatches = 0;
%figure 18
conf.visualize_RT = 1;
%figure 19, 100
conf.visualize_3D = 0;

%debug
conf.debug_main = 1;
conf.debug_minimizer = 0;
conf.debug_matching = 0;
conf.debug_kalman = 1;


% auxiliary image
conf.MAX_R = 5;

%limiting RGB-D depth in cheating EdgeFinder
conf.cheat_lower_bound = 1.5;
conf.cheat_upper_bound = 1e3;

% EstimateQuantile
conf.S_RHO_MIN = 1e-3; % starting uncertainty of histogram in EstimateQuantile
%conf.S_RHO_MIN = 1/5; % starting uncertainty of histogram in EstimateQuantile
%conf.S_RHO_MAX = 20; % final uncertainty of histogram in EstimateQuantile
conf.S_RHO_MAX = 1/0.5; % final uncertainty of histogram in EstimateQuantile
conf.PERCENTILE = 0.9; % quantile threshold in EstimateQuantile
conf.N_BINS = 100; %number of histogram bins in EstimateQuantile
conf.S_RHO_INIT = 1e3; %todo: why 1e3
conf.S_RHO_CUTOFF = 0.1;

% TryVelRot
conf.MATCH_THRESH = 1.; % Match Gradient Treshold
conf.MATCH_NUM_THRESH = 2; % minimal number of frames KL has to be on to be considered stable (exluding very first MATCH_NUM_THRESH frames)
conf.REWEIGHT_DISTANCE = 2.; % Distance cut-off for reweigthing (k_hubber/k_huber)
  

%global tracker
% init inverse depth mean & variance (for normrnd)
%TUM
%
conf.RHO_INIT = 2;
conf.RHO_INIT_VAR = 0.5;
%
%KITTI
%{
conf.RHO_INIT = 10;
conf.RHO_INIT_VAR = 3;
%}

conf.ITER_MAX = 15; % iterations of Farquad probably Eq. (9)
conf.LM_INIT_V = 2; %lm params
conf.TAU = 1e-3; %lm params

%matching
conf.GLOBAL_MATCH_THRESHOLD = 500; %if number of matched KL is below, estimation will be restarted

% DirectedMatching
conf.MATCH_THRESH_MODULE = 1; % Gradient matching threshold on the module
conf.MATCH_THRESH_ANGLE_COS = cos(pi/4); % Gradient matching threshold on the angle
conf.SEARCH_RANGE = 20; % Max Pixel Distance to search for
conf.LOCATION_UNCERTAINTY_MATCH = 2; % Modelled Pixel uncertainty on the matching step

%regularization
conf.REGULARIZE_THRESH = 0.5; % threshold on gradient

%Kalman
conf.RESHAPE_Q_ABSOLUTE = 1e-4; % Constant uncertainty added to depth error
conf.RESHAPE_Q_RELATIVE = 1e-2; % Relative uncertainty added to depth error
conf.LOCATION_UNCERTAINTY = 1; % Location error [px]

%estimate rescaling
conf.MATCH_NUM_MIN = 1; % 
conf.APPLY_SCALE = 0;


end
