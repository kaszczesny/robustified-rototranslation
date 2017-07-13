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

KL = cell(n_frames,1);
img_mask = cell(n_frames,1);
for iter = 1:2
  [KL{iter}, img_mask{iter}] = EdgeFinder(conf.im_name{iter});
end

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

%other fluff
Pos = zeros(3,1); %estimated position
R = eye(3); % rotation matrix
Pose = eye(3); % global rotation
klm_num = 0;
EstimationOk = 1;

for frame=2:2
  % reset before new frame
  RVel = eye(3)*1e50;
  RW0 = eye(3)*1e50; %yup, 1e-10 is never used
  R = eye(3);
  EstimationOk = 1;

  [ ...
    F, ...
    Vel, W0, RVel, RW0, ...
    KL{frame-1}, ...
    rel_error, error_score, FrameCount ...
  ] = GlobalTracker (...
      Vel, W0, ...
      KL{frame-1}, KL{frame}, ...
      rel_error, error_score, FrameCount ...
  );

  % check for minimization erros
  if any(isnan([Vel; W0]))
    RVel = eye(3)*1e50;
    Vel = zeros(3,1);
    
    Kp = 1;
    P_Kp = 1e50;
    
    EstimationOk = 0;
    if conf.debug
      printf("Error in estimation\n");
    end
    
  else
      R0 = RotationMatrix(W0); %forward rotation
      R = R0'; %backward rotation; todo: check
      
      KL{frame-1} = ForwardRotate( KL{frame-1}, R ); % forward match from old edge map to new, using minimization result
      
      %Match from the new EdgeMap to the old one searching on the stereo line
      [klm_num, KL{frame}] = DirectedMatching(...
          Vel, RVel, R, KL{frame-1}, img_mask{frame-1}, KL{frame});
      if klm_num < conf.GLOBAL_MATCH_THRESHOLD && 0 % todo: remove 0
        RVel = eye(3)*1e50;
        Vel = zeros(3,1);
        
        Kp = 1;
        P_Kp = 10;
        
        EstimationOk = 0;
        if conf.debug
          printf("KL match number too low: %4d, keylines: %4d\n", ...
            klm_num, KL{frame}.ctr);
        end
      else
      
        %regularize edgemap
        for i=1:2 % regularize twice
          [r_num, KL{frame}] = Regularize1Iter(KL{frame});
        end
        
        %improve depth using kalman
        [KL{frame}] = UpdateInverseDepthKalman(Vel, RVel, RW0, KL{frame});
        
        % optionally rescale depth
        [KL{frame}, Kp, P_Kp] = EstimateReScaling(KL{frame}); 
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
  
  if conf.debug
    if ~EstimationOk
      printf("Frame #%4d NOK\n", frame);
    else
      printf("Frame #%4d OK\n", frame);
    end
  end
end

% general todo: check sqrts in norms and squares in variances
% todo: more debug messages
