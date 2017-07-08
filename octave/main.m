%% SETUP %%
clear -exclusive OPENCV
if ~exist('OPENCV')
  pkg load image
  setup_opencv
  graphics_toolkit('fltk')
  OPENCV = 1;
end  


KL1 = EdgeFinder(Config().im_name{1});
KL2 = EdgeFinder(Config().im_name{2});

F = 0; %energy based on dot product of distance residuals
Vel = zeros(3,1); %initial translation estimation (3 vector; init with zeros)
W0 = zeros(3,1); %initial rotation estimation (3 vector; init with zeros)
RVel = eye(3)*1e50; %uncertainty Model if the initial Vel estimate will be used as prior (3x3 matrix; init with eye*1e50)
RW0 = eye(3)*1e-10; %uncertainty Model if the initial W0  estimate will be used as prior (3x3 matrix; init with eye*1e-10)
rel_error = 0; %Estimated relative error on the state (init with zero)
error_score = 0; %Estimated relative error on the score (init with 0)
% aka rel_error_score
FrameCount = 0; %number of processed frames


% something with rescaling depth
Kp = 1;
K = 1;
P_Kp = 5e-6;

[ ...
  F, ...
  Vel, W0, RVel, RW0, ...
  KL1, ...
  rel_error, error_score, FrameCount ...
] = GlobalTracker (...
    Vel, W0, RVel, RW0, ...
    KL1, KL2, ...
    rel_error, error_score, FrameCount ...
);

% check for minimization erros
if any(isnan([Vel; W0]))
  RVel = eye(3)*1e50;
  Vel = zeros(3,1);
end
  
  
