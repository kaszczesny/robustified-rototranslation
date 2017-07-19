function conf = Config()
% return config object


conf=struct();

conf.im_name = @(x) sprintf("../data/KITTI/%.6d.png",x-1+50);

conf.scale = 1/3;
persistent imgsize = size(imresize(imread(conf.im_name(1)), conf.scale));
conf.imgsize = imgsize;
conf.principal_point =  [185.2157 607.1928] * conf.scale; %sort of half
conf.zf = 718.856; %todo: zf_x and zf_y?
conf.FPS = 9.65; % todo: get actual value from dataset

  % DoG:
conf.ksize = 13;
%conf.sigma1 = 3; 
%conf.sigma2 = 2;
conf.sigma1 = 1.7818;
conf.sigma2 = 2.30029;

  % Test 1:
conf.thresh_grad = 0.015;
conf.max_img_value = 255;
conf.detector_dog_thresh = 0.095259868922420; %Relation between DoG threshold and Gradient threshold ~1/Sigma0^4
conf.test_1 = @(n2gI) n2gI > (conf.thresh_grad*conf.max_img_value).^2; % n2gI was squared, but didn't work

  % Test 2:
conf.win_s = 2;
conf.PosNegThresh = 0.2;

conf.visualize = 0;
conf.visualize_score = 0;
conf.visualize_matches = 0;
conf.visualize_depth = 1;
conf.debugMain = 1;
conf.debug = 0;

% auxiliary image
conf.MAX_R = 5;

% EstimateQuantile
conf.S_RHO_MIN = 1e-3; % starting uncertainty of histogram in EstimateQuantile
conf.S_RHO_MAX = 20; % final uncertainty of histogram in EstimateQuantile
conf.PERCENTILE = 0.9; % quantile threshold in EstimateQuantile
conf.N_BINS = 100; %number of histogram bins in EstimateQuantile
conf.S_RHO_INIT = 1e3; %todo: why 1e3

% TryVelRot
conf.MATCH_THRESH = 1.; % Match Gradient Treshold
conf.MATCH_NUM_THRESH = 2; % minimal number of frames KL has to be on to be considered stable (exluding very first MATCH_NUM_THRESH frames)
conf.REWEIGHT_DISTANCE = 2.; % Distance cut-off for reweigthing (k_hubber/k_huber)
  

%global tracker
conf.RHO_MAX = 20; % init inverse depth uncertainty; todo: merge with S_RHO_MAX?
conf.RHO_INIT = 1; % init inverse depth

conf.ITER_MAX = 10; % iterations of Farquad probably Eq. (9)
conf.LM_INIT_V = 2; %lm params
conf.TAU = 1e-3; %lm params

%matching
conf.GLOBAL_MATCH_THRESHOLD = 500; %if number of matched KL is below, estimation will be restarted

% DirectedMatching
conf.MATCH_THRESH_MODULE = 1;
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
conf.MATCH_NUM_MIN = 1; % Gradient matching threshold on the module


end
