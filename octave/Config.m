function conf = Config()
% return config object


conf=struct();

conf.im_name = '../data/000012_10.png';
conf.scale = 1/3;
conf.imgsize = [124, 409];
conf.principal_point =  [62.000   204.500]; %half
conf.zf = 1;
%conf.im_name = '../data/IMG_20170402_145917_1.jpg';

  % DoG:
conf.ksize = 13;
%conf.sigma1 = 3; 
%conf.sigma2 = 2;
conf.sigma1 = 1.7818;
conf.sigma2 = 2.30029;

  % Test 1:
conf.thresh_grad = 0.015;
conf.max_img_value = 255;
conf.test_1 = @(n2gI) n2gI > (conf.thresh_grad*conf.max_img_value).^2; % n2gI was squared, but didn't work

  % Test 2:
conf.win_s = 2;
conf.PosNegThresh = 0.2;

conf.visualize = 0;

% auxiliary image
conf.MAX_R = 5;

% EstimateQuantile
conf.S_RHO_MIN = 1e-3; % starting uncertainty of histogram in EstimateQuantile
conf.S_RHO_MAX = 20; % final uncertainty of histogram in EstimateQuantile
conf.PERCENTILE = 0.9; % quantile threshold in EstimateQuantile
conf.N_BINS = 100; %number of histogram bins in EstimateQuantile

% TryVelRot
conf.MATCH_THRESH = 1.; % Match Gradient Treshold
conf.MATCH_NUM_THRESH = 2; % minimal number of frames KL has to be on to be considered stable (exluding very first MATCH_NUM_THRESH frames)
conf.REWEIGHT_DISTANCE = 2.; % Distance cut-off for reweigthing (k_hubber/k_huber)
  

%global tracker
conf.RHO_MAX = 20; % init inverse depth uncertainty
conf.RHO_INIT = 1; % init inverse depth

conf.ITER_MAX = 10; % iterations of Farquad probably Eq. (9)
conf.LM_INIT_V = 2; %lm params
conf.TAU = 1e-3; %lm params

end
