%% SETUP %%
clear
pkg load image
setup_opencv
graphics_toolkit('fltk')


KL1 = EdgeFinder(Config().im_name);

%% AUXILIARY IMAGE %%
[distance_field, KLidx_field] = AuxiliaryImage(KL1);

[ ...
  F, ...
  Vel, W0, RVel, RW0, ...
  rel_error, rel_error_score, ...
  FrameCount ...
] = GlobalTracker (...
    zeros(3,1), ... % Vel; initial translation estimation (3 vector; init with zeros)
    zeros(3,1) , ... %W0; initial rotation estimation (3 vector; init with zeros)
    eye(3)*1e50, %RVel; uncertainty Model if the initial Vel estimate will be used as prior (3x3 matrix; init with eye*1e50)
    eye(3)*1e-10,  %RW0; uncertainty Model if the initial W0  estimate will be used as prior (3x3 matrix; init with eye*1e-10)
    KL1, ...
    0, ... %rel_error;  Estimated relative error on the state (init with zero)
    0, ... %rel_error_score;  Estimated relative error on the score (init with 0)
    distance_field, KLidx_field, ... % Auxilary image results
    0 ... %FrameCount; number of processed frames
);    