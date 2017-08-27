%% SETUP %%
clear -exclusive OPENCV
close all
if ~exist('OPENCV')
  pkg load image
  pkg load optim
  pkg load quaternion
  setup_opencv
  graphics_toolkit('fltk')
  OPENCV = 1;
end  

global frame;
global conf;
conf = Config();
Matcher; % load functions
GlobalTracker;
Visualizer;
Utils;
FitTrajectory;

if conf.save_images
  printf("%s\n\n", conf.output_folder)
  mkdir(conf.output_folder);
  copyfile('Config.m', strcat(conf.output_folder, '/Config.m'));
end  

% arguments/returns for GlobalTracker
F = 0; %energy based on dot product of distance residuals
Vel = zeros(3,1); %initial translation estimation (3 vector; init with zeros)
Vel(3)=1; % this will be used only if not cheating
W0 = zeros(3,1); %initial rotation estimation (3 vector; init with zeros)
RVel = eye(3)*1e50; %uncertainty Model if the initial Vel estimate will be used as prior (3x3 matrix; init with eye*1e50)
RW0 = eye(3)*1e50;  %uncertainty Model if the initial W0  estimate will be used as prior (3x3 matrix; init with eye*1e50)
rel_error = 0; %Estimated relative error on the state (init with zero)
error_score = 0; %Estimated relative error on the score (init with 0)
% aka rel_error_score
FrameCount = 0; %number of processed frames


% something with rescaling depth
Kp = 1;
P_Kp = 5e-6;

if ~conf.cheat
  frame = conf.frame_start-1;
  [KL, img_mask] = EdgeFinder(conf.frame_start, 0);
  % else this will be done in main loop
end  

if conf.TUM
  %TUM-only
  ground_truth_ = dlmread('../data/TUM/groundtruth_corrected.txt', ',');
  ground_truth = zeros(size(ground_truth_), 6);

  %ground truth translation
  ground_truth(:, 1:3) = ground_truth_(:, 1:3);
  ground_truth(:, [1 3 2]) = ground_truth(:, [1 2 3]);

  %ground truth rotation
  quat = quaternion( ...
    ground_truth_(:, 7), ... % ww
    ground_truth_(:, 4), ... % wx
    ground_truth_(:, 6), ... % wz
    ground_truth_(:, 5));... % wy
  ground_truth(:, 4:6) = q2rot(unit(quat))';

  %start in zero
  ground_truth(:,1:3) -= ground_truth(conf.frame_start,1:3);
else
  ground_truth_ = dlmread('../../00/00.txt', ' ');
  ground_truth = zeros(size(ground_truth_,1),6);
  for i = 1:size(ground_truth_,1)
    g = ground_truth_(i,:);
    m = reshape(g', [4 3])';
    l = logm(m(1:3,1:3));
    ground_truth(i,:) = [m(:,4)' -l(1,2) l(1,3) -l(2,3)];
  end
  
  ground_truth(:,1:3) -= ground_truth(conf.frame_start,1:3);
end
%make things easier
frames = conf.frame_start+[conf.frame_interval:conf.frame_interval:conf.n_frames];

%other fluff
Pos = zeros(3,1); %estimated position
R = eye(3); % rotation matrix
Pose = eye(3); % global rotation
klm_num = 0;
EstimationOk = 0;

KL_save = {};
Vel_save = [];
W0_save = [];
RVel_save = [];
RW0_save = [];
Pos_save = [];
Pose_save = [];
gt_save = [];
time_save = [];
ok = [];

%{
im_left = imresize(imread("../../00/image_0/000060.png"), conf.scale);
im_right = imresize(imread("../../00/image_1/000060.png"), conf.scale);

stereo = cv.StereoBM;
disparity = stereo.compute(im_left, im_right);
disparity = double(disparity)/16;

for iter = 1:KL.ctr
  if disparity(KL.pos(iter,1), KL.pos(iter,2)) > 0
    KL.rho(iter,1) = disparity(KL.pos(iter,1), KL.pos(iter,2));
  end
end
%}

sound = wavread('../data/sound.wav');

iiii = 0;
for frame=conf.frame_start+[conf.frame_interval:conf.frame_interval:conf.n_frames]
  iiii++;
  if mod(iiii,10) == 0 && conf.save_images
    save(sprintf("%s/dump.mat", conf.output_folder), 'Pos_save', ...
      'Pose_save', 'RVel_save', 'RW0_save', 'Vel_save', 'W0_save', ...
      'gt_save', 'time_save', 'ok');
  end
  disp('')
  fflush(stdout);
  tic
  
  if conf.cheat
    if ~EstimationOk
      printf("Reestimating frame %d\n", frame-conf.frame_interval);
      
      % redo edge finding, this time with RGBD depth
      [KL, img_mask] = EdgeFinder(frame-conf.frame_interval, 1);
      
      KL.frames += 1;

      % acquire VelRot from ground truth
      Vel = ground_truth(frame-conf.frame_interval, 1:3)';
      W0 = ground_truth(frame-conf.frame_interval, 4:6)';
    end
  end
  
  KL_save{end+1} = KL;
  KL_prev = KL;
  img_mask_prev = img_mask;
  
  [KL, img_mask] = EdgeFinder(frame, 0);

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
  ] = MinimizeRV (...
      Vel, W0, ...
      KL_prev, KL, ...
      rel_error, error_score, FrameCount ...
  );
  fflush(stdout);
  
  Vel_save = cat(2, Vel_save, Vel);
  W0_save = cat(2, W0_save, W0);
  RVel_save = cat(3, RVel_save, RVel);
  RW0_save = cat(3, RW0_save, RW0);

  % check for minimization erros
  if any(isnan([Vel; W0]))
    RVel = eye(3)*1e50;
    Vel = zeros(3,1);
    Vel(3)=1;
    
    Kp = 1;
    P_Kp = 1e50;
    
    FrameCount = 0;
    EstimationOk = 0;
    if conf.debug_main
      printf("frame #%4d: Error in estimation, KLs: %d, %d\n", frame,
        KL_prev.ctr, KL.ctr);
    end
    
    Pos_save = cat(2, Pos_save, Pos);
    Pose_save = cat(3, Pose_save, Pose);
    if conf.visualize_RT  
      gt_now = ground_truth(frame, 1:3);;
      gt_save = cat(1, gt_save, gt_now);
    end
    
    ok(end+1) = 0;
    
    continue
    
  else
      R0 = RotationMatrix(W0); %forward rotation
      % in C++ there is a multiplication by R, but at this point R=eye(3)
      R = R0'; %backward rotation;
      
      KL_prev = ForwardRotate( KL_prev, R' );
      
      % forward match from old edge map to new, using minimization result
      [klm_num_forward, KL] = ForwardMatch(KL, KL_prev);
      
      %Match from the new EdgeMap to the old one searching on the stereo line
      [klm_num, KL] = DirectedMatching(...
          Vel, RVel, R, KL_prev, img_mask_prev, KL);
      [klm_num_forward klm_num]
      
      if klm_num < conf.GLOBAL_MATCH_THRESHOLD
        RVel = eye(3)*1e50;
        Vel = zeros(3,1);
        Vel(3)=1;
        
        Kp = 1;
        P_Kp = 10;
        
        FrameCount = 0;
        EstimationOk = 0;
        if conf.debug_main
          printf("frame #%4d: KL match number too low: %4d, keylines: %4d\n", ...
            frame, klm_num, KL.ctr);
        end
        
        Pos_save = cat(2, Pos_save, Pos);
        Pose_save = cat(3, Pose_save, Pose);
        if conf.visualize_RT  
          gt_now = ground_truth(frame, 1:3);
          gt_save = cat(1, gt_save, gt_now);
        end
        
        ok(end+1) = 0;
        
        continue
        
        
      else
      
        % regularize only after median filter has been applied once
        if FrameCount > 1
          %regularize edgemap
          for i=1:2 % regularize twice
            [r_num, KL] = Regularize1Iter(KL);
          end
        end
        
        %improve depth using kalman
        [KL] = UpdateInverseDepthKalman(Vel, RVel, RW0, KL);
        
        if FrameCount > 20
          % optionally rescale depth
          [KL, Kp, P_Kp] = EstimateReScaling(KL);
        end  
        
      end
  end
  
  if 1
    dt_frame = 1./ conf.FPS;
  else
    % todo: dt_frame = current frame time - previous frame time
  end

  %estimate position and pose
  Pose = Pose * R;
  Pos_prev = Pos;
  Pos += -Pose * Vel;
  Pos_save = cat(2, Pos_save, Pos);
  Pose_save = cat(3, Pose_save, Pose);
  ok(end+1) = 1;
  
  % RVel = RVel ./ (dt_frame.^2); % quite no point in doing that
  
  if conf.debug_main
    if ~EstimationOk
      printf("Frame #%4d NOK\n", frame);
      soundsc(sound,44.1e3,16,[-50.0,50.0]);
      %keyboard("<<<")
    else
      printf("Frame #%4d OK (since %d frames)\n", frame, FrameCount-1);
    end
  end
  
  if conf.visualize_RT  
    gt_now = ground_truth(frame, 1:3);
    gt_save = cat(1, gt_save, gt_now);
    %{
    gt_now = ground_truth(frame, 1:3).*scalegt;
    gt_now([1 3]) *= Rgt;
    
    ;
    
    %dont draw anything until init is finished (few frames)
    if (frame - conf.frame_start)/conf.frame_interval == 8
      figure(18);
      title('RT')
      hold on
      grid on
      
      %get new zero coordinate
      pos_zero = Pos;
      gt_zero = gt_now;
      %draw everything up until now
      for iter = 2:8
        plot( [Pos_save(1, iter-1) Pos_save(1, iter)]-pos_zero(1), ...
              [Pos_save(3, iter-1) Pos_save(3, iter)]-pos_zero(3), 'rx-');
        plot( [gt_save(iter-1,1) gt_save(iter,1)]-gt_zero(1), ...
              [gt_save(iter-1,3) gt_save(iter,3)]-gt_zero(3), 'b+-')
      end
    end
    if (frame - conf.frame_start)/conf.frame_interval > 8
      figure(18)
      hold on
      grid on
      
      plot( [Pos_prev(1) Pos(1)]-pos_zero(1), ...
            [Pos_prev(3) Pos(3)]-pos_zero(3), 'rx-');
      plot( [gt_save(end-1,1) gt_save(end,1)]-gt_zero(1), ...
            [gt_save(end-1,3) gt_save(end,3)]-gt_zero(3), 'b+-')
    end
    
    %plot(ground_truth(frame+[-conf.frame_interval 0], 1), ...
    %  ground_truth(frame+[-conf.frame_interval 0], 3), 'b+-')
    %plot( ...
    %  Pos(1) + [0 ground_truth(frame,1)-ground_truth(frame-conf.frame_interval,1)], ...
    %  Pos(3) + [0 ground_truth(frame,3)-ground_truth(frame-conf.frame_interval,3)], ...
    %  'go-')
    hold off
    pause(0)
    %}
    if size(Pos_save,2) > 10
      [scale, score, R, T]=FitTrajectory_(gt_save(5:end,:)', Pos_save(:,5:end))
    end
  end
  
  
  VisualizeMatches(KL_prev, KL, 0);
  VisualizeMatches(KL_prev, KL, 1);
  
  if conf.visualize_3D
    KL = VisualizeDepth3D(KL); %median filter is in there, so better get this done before vis_depth
  end
  
  
  VisualizeDepth(KL);
  VisualizeDepthVar(KL);
   
  VisualizeHistory(KL);
  
  if any(isnan(KL_prev.rho(:))) && conf.debug_main
    printf("rho is nan\n");
  end
  
  time_save(end+1) = toc;
  time_elapsed = time_save(end)
  
  if exist('stop', 'file') > 0
    soundsc(sound,44.1e3,16,[-50.0,50.0]);
    keyboard("<<<") % type "return" to continue
  end
  
  if any(KL_prev.rho(:,1) < 0)
    printf('rho negative %d\n', sum(KL_prev.rho(:,1) < 0))  
    %keyboard("<<<")
  end  
end
