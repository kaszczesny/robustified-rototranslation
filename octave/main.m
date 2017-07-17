%% SETUP %%
clear -exclusive OPENCV
if ~exist('OPENCV')
  pkg load image
  setup_opencv
  graphics_toolkit('fltk')
  OPENCV = 1;
end  

conf = Config();
Matcher; % load functions

n_frames = 10;

% arguments/returns for GlobalTracker
F = 0; %energy based on dot product of distance residuals
Vel = zeros(3,1); %initial translation estimation (3 vector; init with zeros)
W0 = zeros(3,1); %initial rotation estimation (3 vector; init with zeros)
RVel = eye(3)*1e50; %uncertainty Model if the initial Vel estimate will be used as prior (3x3 matrix; init with eye*1e50)
RW0 = eye(3)*1e50;  %uncertainty Model if the initial W0  estimate will be used as prior (3x3 matrix; init with eye*1e50)
rel_error = 0; %Estimated relative error on the state (init with zero)
error_score = 0; %Estimated relative error on the score (init with 0)
% aka rel_error_score
FrameCount = 0; %number of processed frames


% something with rescaling depth
Kp = 1;
K = 1;
P_Kp = 5e-6;

[KL, img_mask] = EdgeFinder(conf.im_name(1));

%other fluff
Pos = zeros(3,1); %estimated position
R = eye(3); % rotation matrix
Pose = eye(3); % global rotation
klm_num = 0;
EstimationOk = 1;

KL_save = {};
Vel_save = [];
W0_save = [];
RVel_save = [];
RW0_save = [];

for frame=2:2
  KL_save{end+1} = KL;
  KL_prev = KL;
  img_mask_prev = img_mask;
  
  [KL, img_mask] = EdgeFinder(conf.im_name(frame));

  % reset before new frame
  RVel = eye(3)*1e50;
  RW0 = eye(3)*1e50; %yup, 1e-10 is never used
  R = eye(3);
  EstimationOk = 1;

  [ ...
    F, ...
    Vel, W0, RVel, RW0, ...
    KL_prev, ...
    rel_error, error_score, FrameCount ...
  ] = GlobalTracker (...
      Vel, W0, ...
      KL_prev, KL, ...
      rel_error, error_score, FrameCount ...
  );
  
  Vel_save = cat(2, Vel, Vel_save);
  W0_save = cat(2, W0, W0_save);
  RVel_save = cat(3, RVel, RVel_save);
  RW0_save = cat(4, RW0, RW0_save);

  % check for minimization erros
  if any(isnan([Vel; W0]))
    RVel = eye(3)*1e50;
    Vel = zeros(3,1);
    
    Kp = 1;
    P_Kp = 1e50;
    
    EstimationOk = 0;
    if conf.debugMain
      printf("frame #%4d: Error in estimation\n", frame);
    end
    
  else
      R0 = RotationMatrix(W0); %forward rotation
      R = R0'; %backward rotation;
      
      KL_prev = ForwardRotate( KL_prev, R' );
      
      % forward match from old edge map to new, using minimization result
      [~, KL] = ForwardMatch(KL, KL_prev);
      
      %Match from the new EdgeMap to the old one searching on the stereo line
      [klm_num, KL] = DirectedMatching(...
          Vel, RVel, R, KL_prev, img_mask_prev, KL);
      if klm_num < conf.GLOBAL_MATCH_THRESHOLD
        RVel = eye(3)*1e50;
        Vel = zeros(3,1);
        
        Kp = 1;
        P_Kp = 10;
        
        EstimationOk = 0;
        if conf.debugMain
          printf("frame #%4d: KL match number too low: %4d, keylines: %4d\n", ...
            frame, klm_num, KL.ctr);
        end
      else
      
        %regularize edgemap
        for i=1:2 % regularize twice
          [r_num, KL] = Regularize1Iter(KL);
        end
        
        %improve depth using kalman
        [KL] = UpdateInverseDepthKalman(Vel, RVel, RW0, KL);
        
        % optionally rescale depth
        [KL, Kp, P_Kp] = EstimateReScaling(KL); 
      end
  end
  
  if 1
    dt_frame = 1./ conf.FPS;
  else
    % todo: dt_frame = current frame time - previous frame time
  end

  %estimate position and pose
  Pose = Pose * R;
  Pos += -Pose * Vel * K;
  
  % RVel = RVel ./ (dt_frame.^2); % quite no point in doing that
  
  if conf.debugMain
    if ~EstimationOk
      printf("Frame #%4d NOK\n", frame);
    else
      printf("Frame #%4d OK\n", frame);
    end
  end
  
  
  if(conf.visualize)
    figure()
    pos1 =[];
    pos2 =[];
    im1 = imresize(imread(conf.im_name(frame-1)), conf.scale);
    im2 = imresize(imread(conf.im_name(frame)), conf.scale);
    imsize = size(im1);

    for iter = 1:KL.ctr
    if KL.matching(iter) != -1
      pos1(end+1,:) = KL_prev.pos(KL.matching(iter),:);
      pos2(end+1,:) = KL.pos(iter,:);
      end
    end

    im_plot = [im1, im2; im2, zeros(imsize)];
    imshow(im_plot);
    hold on;

    for iter = 1:length(pos1)
      color = rand(1,3);
      plot([pos1(iter,2), pos2(iter,2)+imsize(2)], ...
            [pos1(iter,1), pos2(iter,1)], "-o", ...
            'Color', color, 'markerfacecolor', color);
      plot([pos1(iter,2), pos2(iter,2)], ...
            [pos1(iter,1), pos2(iter,1)+imsize(1)], "-o", ...
            'Color', color, 'markerfacecolor', color);
    end
    hold off;
  end
end

% general todo: check sqrts in norms and squares in variances
% todo: more debug messages
