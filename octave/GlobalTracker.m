function [ ...
  F, ...
  Vel, W0, RVel, RW0, ...
  KL, ...
  rel_error, rel_error_score, ...
  FrameCount ...
] = GlobalTracker (...
    Vel, ... %initial translation estimation (3 vector; init with zeros)
    W0, ... %initial rotation estimation (3 vector; init with zeros)
    RVel, %uncertainty Model if the initial Vel estimate will be used as prior (3x3 matrix; init with eye*1e50)
    RW0,  %uncertainty Model if the initial W0  estimate will be used as prior (3x3 matrix; init with eye*1e-10)
    KL_prev, ...
    KL, ...
    rel_error, ... % Estimated relative error on the state (init with zero)
    rel_error_score, ... % Estimated relative error on the score (init with 0)
    FrameCount ... % number of processed frames
)
  % based on Minimizer_RV
  % performs Equation (8) minimization
  % returns total energy (?)
  % todo: Check for minimization errors
  
function [s_rho] = EstimateQuantile(KL)
  % based on edge_tracker EstimateQuantile
  % tells which s_rho (uncertainty) is such that S_RHO_MIN-s_rho == quantile
  
  
  bins = zeros(conf.N_BINS, 1);

  for iter = 1:size(KL.rho, 1)
    %check in which bin s_rho is and increment its counter
    idx = floor( conf.N_BINS * (KL.rho(iter,2)-conf.S_RHO_MIN) / (conf.S_RHO_MAX - conf.S_RHO_MIN) ) + 1;
    if idx < 1
      idx = 1;
    elseif idx > conf.N_BINS
      idx = conf.N_BINS;
    end  
    bins(idx) += 1;
  end

  s_rho = conf.S_RHO_INIT;
  a = 0; %accumulator

  for iter = 1:conf.N_BINS
    if (a > conf.PERCENTILE * size(KL.rho, 1) )
      s_rho = (iter - 1) * (conf.S_RHO_MAX-conf.S_RHO_MIN) / conf.N_BINS + conf.S_RHO_MIN;
      return
    end
    a += bins(iter);
  end
end

function R = RotationMatrix(w)
  % https://pixhawk.org/_media/dev/know-how/jlblanco2010geometry3d_techrep.pdf
  % https://github.com/edrosten/TooN/blob/master/so3.h#L254
  % w is a vector that defines rotation axis; it's length is the rotation angle
  
  skewSymmetric = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
  R = expm(skewSymmetric);
  return 
end

function [...
    score, ...
    JtJ, JtF, KL_prev, ...
    P0m, DResidualNew ...
] = TryVelRot( ...
    ReWeight, ... % Rewighting switch
    ProcJF, ... % Calculate Jacobians or just energy score?
    ... % UsePriors is always false
    ... % JtJ, ... % Estimated Jacobian Matrix 6x6 - only returned
    ... % JtF, ... % Estimated Residual Matrix 6x1 - only returned
    VelRot, ... % Proposed state vector 6x1 [Translation Rotation]
    V0p, ... % Translation Model 3x1
    PV0, ... % Model uncertainty 3x3
    W0p, ... % Rotation Model 3x1
    PW0, ... % Model uncertainty 3x3
    KL_prev, KL, ... % keylines
    P0m, ... % linear transpose coodinates vector of 3D klist positions (vector or pnum*3)
    ... % pnum, ... % number of points
    max_s_rho, ... % estimated rho 0.9 quantile
    DResidual, ... % Last iteration Distance Residuals - for calculating weights, vector pnumx1
    ... % DResidualNew, ... % New iteration Distance Residuals - new residuals, vector pnumx1
    distance_field, KLidx_field ...
)
  % returns energy based on dot product of distance residuals
  
  pnum = size(KL_prev.pos,1);
  score = 0;
  JtJ = zeros(6,6);
  JtF = zeros(6,1);
  DResidualNew = DResidual * 0;
  
  % rotations
  R0 = RotationMatrix(VelRot([4:6]));
  RM = RotationMatrix([0 0 VelRot(3)]); % rotation about z-axis
  RM = RM(1:2, 1:2);
  
  mnum = 0; %match counter
  
  %{
    linear transpose coodinates vectors (LTCV) are arranged as follows:
    {x1 x2 x3 .... xn y1 y2 y3 ... yn z1 z2 z3 ... zn}
    where x & y are KL subpixel position in homogeneous coordinates (plane on focal length zf)
    and z is rho (estimated inverse depth)
    Then they are projected into 3D by Equation 4
  %}
  
  conf = Config();
  
  % Rt is just flattened R0, Vt is Velocity (1:3) component of VelRot
  
  Ptm = zeros(pnum, 3); %LTCV of transformed 3d coordinates
  PtIm = zeros(pnum, 3); %LTCV of transformed image coordinates
  
  fm = zeros(pnum, 1); %weighted residuals
  df_dPi = zeros(pnum, 2); %weighted img derivative of residual
  
  % perform SE3: Pos = Rot*Pos0 + Vel
  Ptm = (R0 * P0m')';
  for iter=1:3
    Ptm(:, iter) += VelRot(iter);
  end
  
  % equation 3: 3d -> image
  PtIm(:,3) = 1 ./ Ptm(:,3);
  Pz_zf = conf.zf * PtIm(:,3);
  PtIm(:,2) = Ptm(:,2) .* Pz_zf;
  PtIm(:,1) = Ptm(:,1) .* Pz_zf;
  
  % klist in C in actually old keymap,
  % while auxiliary field holds indexes for current keymap
  
  for iter = 1:pnum
    KL_prev.forward = -1;
    
    % don't use this keyline if uncertainty is too high
    % or if  keyline hasn't appeared in at least 2 consecutive frames
    % (exluding init)
    if KL_prev.rho(iter,2) > max_s_rho || ...
       KL_prev.frames < min(conf.MATCH_NUM_THRESH, FrameCount)

      fm(iter) = 0;
      df_dPi(iter, :) = 0;
      continue;
      
    end
    
    % convert to image coordinates (just add p.p.)
    p_pji_y = PtIm(iter, 1) + conf.principal_point(1); 
    p_pji_x = PtIm(iter, 2) + conf.principal_point(2); 
    
    x = round(p_pji_x);
    y = round(p_pji_y);
    
    %estimate reweighting
    weight = 1;
    if ReWeight && abs(DResidual(iter)) > conf.REWEIGHT_DISTANCE
      weight = conf.REWEIGHT_DISTANCE ./ abs(DResidual(iter));
    end
    
    %if outside border, consider it a mismatch
    if ( x<2 || y<2 || x>=conf.imgsize(2) || y>=conf.imgsize(1) )
      fm(iter) = conf.MAX_R/KL_prev.rho(iter, 2);
      
      if(ReWeight)
        fm(iter) *= weight;
      end
      df_dPi(iter, :) = 0;
      DResidualNew(iter) = conf.MAX_R;
      continue;
    end
    
    %temporairly rotate gradient on z axis for improved matching (?)
    kl_m_m_copy = KL_prev.grad(iter, :);
    
    KL_prev.grad(iter,1) = RM(2,1) * kl_m_m_copy(2) + RM(2,2) * kl_m_m_copy(1); % todo: possibly fucked up matrix coordinates
    KL_prev.grad(iter,2) = RM(1,1) * kl_m_m_copy(2) + RM(1,2) * kl_m_m_copy(1);
    
    % Calc_f_J was here
    
    % f_inx corresponds to (y,x)
    
    kl_iter = KLidx_field(y,x); % index from current frame
    if kl_iter < 0
      df_dPi(iter, :) = 0;
      fm(iter) = max_r ./ KL.rho(2);
      fi = 0;
    else
      % Test_f_k was here - a quick test for KL match
      % f_m - gradient from current frame (extraced from aux)
      % kl - keyline from previous frame
      m1 = KL.grad(kl_iter, :);
      m2 = KL_prev.grad(iter, :);
      
      m2_norm_sq = dot(m2, m2);
      pablo_escobar = dot(m1, m2); 
      
      if abs(pablo_escobar - m2_norm_sq) > conf.MATCH_THRESH * m2_norm_sq
        df_dPi(iter, :) = 0;
        fm(iter) = max_r ./ KL.rho(2);
        fi = 0;
      else
        d = [p_pji_y p_pji_x] - KL.posSubpix(kl_iter,:);
        
        u_m = KL.grad(kl_iter, :);
        u_m ./ sqrt(dot(u_m,u_m)); % normalized gradient
        
        fi = dot(d, u_m); %residual in direction of the gradient
        
        df_dPi(iter, :) = u_m ./ KL.rho(iter,2);
        
        mnum += 1;
        KL_prev.forward(iter) = kl_iter;
        fm(iter) = fi ./ KL_prev.grad(iter, 2);
        
      end  
    end
    
    KL_prev.grad(iter,:) = kl_m_m_copy;
    
    if(ReWeight)
      fm(iter) *= weight;
      df_dPi(iter, :) *= weight;
    end
    
    DResidualNew(iter) = fi;
    
  end
  
  if(ProcJF)
    
    Jm = zeros(pnum, 6);
    RhoTmp0 = zeros(pnum, 1);
    
    RhoTmp0 = conf.zf * PtIm(:, 3); %z coordinate constant mul
    
    %x
    Jm(:,2) = RhoTmp0 .* df_dPi(:,2);
    %y
    Jm(:,1) = RhoTmp0 .* df_dPi(:,1);
    %z-x
    RhoTmp0 = PtIm(:,3) .* PtIm(:,2);
    Jm(:,3) = RhoTmp0 .* df_dPi(:,2);
    %z-y
    RhoTmp0 = PtIm(:,3) .* PtIm(:,1);
    Jm(:,3) += RhoTmp0 .* df_dPi(:,1);
    
    %x
    Jm(:,5) = Jm(:,1) .* Ptm(:,3);
    Jm(:,5) += Jm(:,3) .* Ptm(:,1);
    %y
    Jm(:,4) = Jm(:,2) .* Ptm(:,3);
    Jm(:,4) += Jm(:,3) .* Ptm(:,2);
    %z
    RhoTmp0 = Jm(:,2) .*Ptm(:,1);
    Jm(:,6) = -1 * RhoTmp0;
    Jm(:,6) += Jm(:, 1) .* Ptm(:,2);
    
    %there should be a fragment to 0 the non-4-divisible knum, but due to no need for it in octave we'll omit this part
    
    %dot product on half of the JtJ
    for iter=1:6
      for jter=iter:6
        JtJ(iter, jter) = dot(Jm(:,iter), Jm(:,jter));
      end
      JtF(iter) = dot(Jm(:,iter), fm);
    end
    
    %wrongly estimated signs (?) I DONT KNOW IF ITS CORRECT, JACOBIANS SCARE ME
    for iter=1:2
      JtF(iter+2) *= -1;
      for jter=1:2
        JtJ(iter, jter+2) *= -1;
        JtJ(iter+2, jter+4) *= -1;
      end
    end
    
    %symmetrize the matrix
    for iter=1:6
      for jter=iter+1:6
        JtJ(jter, iter) = JtJ(iter, jter);
      end
    end
    
    %again, ommiting the part where its not divisible by 4
    
    
  end
 
  score = dot(fm, fm); %dot product
 
  %if(UsePriors)
    % doesn't matter, aways false
  %end
                                
end

%%%%%%%%%%%%%% Minimizer_RV starts here %%%%%%%%%%%%%%
                            
  %% AUXILIARY IMAGE %%
  [distance_field, KLidx_field] = AuxiliaryImage(KL);
  
  % UsePriors is always false
  conf = Config();
  % init_type is always 2
  max_s_rho = EstimateQuantile(KL_prev);
  INIT_ITER = 2; % Actually controls ProcJF in TryVelRot (true or false depending on interation)
  

  if size(KL_prev.idx , 1) < 1
    F = 0;
    return
  end

  JtJ = zeros(6,6); % jacobian
  ApI = zeros(6,6);
  JtJnew = zeros(6,6);
  
  JtF = zeros(6,1); % estimated residuals 
  JtFnew = zeros(6,1);
  
  h = zeros(6,1); %sth with JtJ, SVD and multiplying by JtF
  X = zeros(6,1);
  Xnew = zeros(6,1);
  Xt = zeros(6,1);
  
  pnum = size(KL_prev.idx , 1);
  
  P0Im = zeros(pnum, 3); %image coordinates
  P0m = zeros(pnum, 3); %3d coordinates
  
  Residual = zeros(pnum, 1); %Res0;distance residuals
  ResidualNew = zeros(pnum, 1);  %Res1;
  Rest = zeros(pnum, 1);
  
  %converto to ltcv
  P0Im(:,1) = KL_prev.posSubpix(:,1) + conf.principal_point(1); % y
  P0Im(:,2) = KL_prev.posSubpix(:,2) + conf.principal_point(2); % x
  P0Im(:,3) = KL_prev.rho(:,1);
  
  %proyect (eq 4.) imgage -> 3d
  P0m(:,3) = 1./ P0Im(:,3); % depth from inverse depth
  Pz_zf = 1./conf.zf * P0m(:,3);
  P0m(:,1) = P0Im(:,1) .* Pz_zf; %y
  P0m(:,2) = P0Im(:,2) .* Pz_zf; %x
  
  F = 0; %energy scores
  Fnew = 0;
  F0 = 0;
  
  v = conf.LM_INIT_V;
  
  u = 0;
  gain = 0;
  
  eff_steps = 0; %count how many times we gained
  
  %zero init
  [F, JtJ, JtF, KL_prev, P0m, ResidualNew] = TryVelRot(
    0,1,X,Vel,RVel, W0, RW0, 
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field);
  F0 = F; 
  u = conf.TAU * max(max(JtJ));
  
  for iter = 1:INIT_ITER
    ApI = JtJ + eye(6) * u;
    [U, D, Vt] = cv.SVD.Compute(ApI); 
    h = cv.SVD.BackSubst(U, D, Vt, -JtF); % back substitution
   
    Xnew = X+h;
    
    if iter == INIT_ITER
      ProcJF = 0;
    else
      ProcJF = 1;
    end    
    
    [Fnew, JtJ, JtF, KL_prev, P0m, ResidualNew] = TryVelRot(
      0,ProcJF,X,Vel,RVel, W0, RW0, 
      KL_prev, KL, P0m,
      max_s_rho,Residual, distance_field, KLidx_field);
      
    if iter == INIT_ITER
      gain = F - Fnew;
    else
      gain = (F-Fnew)/(0.5 * h' * (u*h-JtF));
    end     
    
    if gain > 0
      F=Fnew;
      X=Xnew;
      JtJ = JtJnew;
      JtF = JtFnew;
      u *= max( 0.33, 1-((2*gain-1)^3));
      v = 2;
      eff_steps++;
    else
      u *= v;
      v *= 2;
    end
  end
  
  Xt = X;
  Ft = F;
  F0t = F0;
  ut = u;
  vt = v;
  eff_steps_t = eff_steps;
  
  eff_steps = 0;
  
  
  X = [Vel; W0]; %usePriors
  [F, JtJ, JtF, KL_prev, P0m, ResidualNew] = TryVelRot(
    0,1,X,Vel,RVel, W0, RW0, 
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field);
  F0 = F; 
  u = conf.TAU * max(max(JtJ));
  v = 2;
  
  for iter = 1:INIT_ITER
    ApI = JtJ + eye(6) * u;
    [U, D, Vt] = cv.SVD.Compute(ApI); 
    h = cv.SVD.BackSubst(U, D, Vt, -JtF); % back substitution
   
    Xnew = X+h;
    
    if iter == INIT_ITER
      ProcJF = 0;
    else
      ProcJF = 1;
    end   
    
    [Fnew, JtJ, JtF, KL_prev, P0m, ResidualNew] = TryVelRot(
      0,ProcJF,X,Vel,RVel, W0, RW0, 
      KL_prev, KL, P0m,
      max_s_rho,Residual, distance_field, KLidx_field);
      
    if iter == INIT_ITER
      gain = F - Fnew;
    else
      gain = (F-Fnew)/(0.5 * h' * (u*h-JtF));
    end  
    
    if gain > 0
      F=Fnew;
      X=Xnew;
      JtJ = JtJnew;
      JtF = JtFnew;
      u *= max( 0.33, 1-((2*gain-1)^3));
      v = 2;
      eff_steps++;
    else
      u *= v;
      v *= 2;
    end
  end
  
  if F>Ft
    X = Xt;
    F = Ft;
    F0 = F0t;
    u = ut;
    v = vt;
    eff_steps = eff_steps_t;
    ResidualNew = Rest;
  end
  
  tRes = Residual;
  Residual = ResidualNew;
  ResidualNew = tRes;
  
  %reweight
  [F, JtJ, JtF, KL_prev, P0m, ResidualNew] = TryVelRot(
    1,1,X,Vel,RVel, W0, RW0, 
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field);
  F0 = F; 
  u = conf.TAU * max(max(JtJ));
  v = 2;
  
  for iter = 1:conf.ITER_MAX
    ApI = JtJ + eye(6) * u;
    
    %todo: Cholesky
    %[R, P, Q] = chol(ApI);
    [U, D, Vt] = cv.SVD.Compute(ApI); 
    h = cv.SVD.BackSubst(U, D, Vt, -JtF); % back substitution
    
    Xnew = X+h;
    [Fnew, JtJnew, JtFnew, KL_prev, P0m, ResidualNew] = TryVelRot(
      1,1,Xnew,Vel,RVel, W0, RW0, 
      KL_prev, KL, P0m,
      max_s_rho,Residual, distance_field, KLidx_field);
    
    gain = (F-Fnew)/(0.5*h'*(u*h-JtF));
    if gain > 0
      F=Fnew;
      X=Xnew;
      JtJ = JtJnew;
      JtF = JtFnew;
      u *= max( 0.33, 1-((2*gain-1)^3));
      v = 2;
      eff_steps++;
      
      tRes = Residual;
      Residual = ResidualNew;
      ResidualNew = tRes;
    else
      u *= v;
      v *= 2;
    end
  end
  
  %todo: Cholesky
  %[R, P, Q] = chol(JtJ);
  %todo: RRV
  RRV = inv(JtJ);
  
  Vel = X(1:3);
  W0 = X(4:6);
  
  RVel = RRV(1:3, 1:3);
  RW0 = RRV(4:6,4:6);
  
  if eff_steps>0
    norm_h = h/sqrt(dot(h));
    norm_X = X/sqrt(dot(X));
    rel_error = norm_h/(norm_X + 1e-30);
    rel_error_score = F/F0;
  else
    rel_error = 1e20;
    rel_error_score = 1e20;
  end
  
  FrameCount++;
  
end
