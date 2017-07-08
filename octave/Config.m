function conf = Config()
% return config object


conf=struct();

conf.im_name = {
  '../data/KITTI/000050.png'
  '../data/KITTI/000051.png'
  '../data/KITTI/000052.png'
  '../data/KITTI/000053.png'
  '../data/KITTI/000054.png'
  '../data/KITTI/000055.png'
  '../data/KITTI/000056.png'
  '../data/KITTI/000057.png'
  '../data/KITTI/000058.png'
  '../data/KITTI/000059.png'
}  ;
conf.scale = 1/3;
conf.imgsize = [124, 409]; %todo: don't hardcode
conf.principal_point =  [185.2157 607.1928] * conf.scale; %sort of half
conf.zf = 718.856; %todo: zf_x and zf_y?
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
conf.debug = 1;

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
conf.RHO_MAX = 20; % init inverse depth uncertainty
conf.RHO_INIT = 1; % init inverse depth

conf.ITER_MAX = 10; % iterations of Farquad probably Eq. (9)
conf.LM_INIT_V = 2; %lm params
conf.TAU = 1e-3; %lm params

end
