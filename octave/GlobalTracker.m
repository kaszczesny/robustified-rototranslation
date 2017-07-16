function [ ...
  F, ...
  Vel, W0, RVel, RW0, ...
  KL_prev, ...
  rel_error, rel_error_score, ...
  FrameCount ...
] = GlobalTracker (...
    Vel, ... %initial translation estimation (3 vector; init with zeros)
    W0, ... %initial rotation estimation (3 vector; init with zeros)
    ... % RVel, %uncertainty Model if the initial Vel estimate will be used as prior (3x3 matrix; init with eye*1e50) - actually only returned
    ... % RW0,  %uncertainty Model if the initial W0  estimate will be used as prior (3x3 matrix; init with eye*1e-10) - actually only returned (and actually 1e50)
    KL_prev, ...
    KL, ...
    rel_error, ... % Estimated relative error on the state (init with zero)
    rel_error_score, ... % Estimated relative error on the score (init with 0)
    FrameCount ... % number of processed frames
)
  % based on Minimizer_RV
  % performs Equation (8) minimization
  % returns total energy (?)
  
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

function [...
    score, ...
    JtJ, JtF, ...
    KL_prev_forward, ... % only forward field is updated
    DResidualNew ...
] = TryVelRot( ...
    ReWeight, ... % Rewighting switch
    ProcJF, ... % Calculate Jacobians or just energy score?
    ... % UsePriors is always false
    ... % JtJ, ... % Estimated Jacobian Matrix 6x6 - only returned
    ... % JtF, ... % Estimated Residual Matrix 6x1 - only returned
    VelRot, ... % Proposed state vector 6x1 [Translation Rotation]
    V0p, ... % Translation Model 3x1
    ... % PV0, ... % Model uncertainty 3x3 - actually not used
    W0p, ... % Rotation Model 3x1
    ... % PW0, ... % Model uncertainty 3x3 - actually not used
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
  
  % perform SE3: Pos = Rot*Pos0 + Vel     (eq. 5 an 2)
  Ptm = (R0 * P0m')';
  for iter=1:3
    Ptm(:, iter) += VelRot(iter);
  end
  
  % equation 3: 3d -> image
  % from the paper: "Points from the previous edge-map (q_0, rho_0) are projected
  %                  into the points (q_t, rho_t) by using the warping function tau.
  PtIm(:,3) = 1 ./ Ptm(:,3);
  Pz_zf = conf.zf * PtIm(:,3);
  PtIm(:,2) = Ptm(:,2) .* Pz_zf;
  PtIm(:,1) = Ptm(:,1) .* Pz_zf;
  
  % the klist argument in C is actually OLD keymap,
  % while this->auxiliary field holds indexes for CURRENT keymap
  
  for iter = 1:pnum
    KL_prev.forward(iter) = -1; % reset forward match
    
    % don't use this keyline if uncertainty is too high
    % or if  keyline hasn't appeared in at least 2 consecutive frames (todo: investigate how that works)
    % (exluding init)
    if KL_prev.rho(iter,2) > max_s_rho || ...
       KL_prev.frames(iter) < min(conf.MATCH_NUM_THRESH, FrameCount)

      if conf.debug
        if KL_prev.rho(iter,2) > max_s_rho
          printf('KL #%4d @frame #%4d: rho uncertainty too high: %f\n', ...
            iter, KL_prev.frame_id, KL_prev.rho(iter,2))
        else
          printf('KL #%4d @frame #%4d: has not appreared(%d, %d)\n', ...
            iter, KL_prev.frame_id, KL_prev.frames(iter), FrameCount)
        end
      end

      fm(iter) = 0;
      df_dPi(iter, :) = 0;
      continue;

    end
    
    % convert to image coordinates (just re-add p.p.): q_t
    p_pji_y = PtIm(iter, 1) + conf.principal_point(1); 
    p_pji_x = PtIm(iter, 2) + conf.principal_point(2); 
    
    x = round(p_pji_x);
    y = round(p_pji_y);
    
    %estimate reweighting
    weight = 1;
    if ReWeight && abs(DResidual(iter)) > conf.REWEIGHT_DISTANCE
      weight = conf.REWEIGHT_DISTANCE ./ abs(DResidual(iter)); % Hubber norm
      % it isn't squared, because fm is squared with a dot product at the end of the function
    end
    
    %if outside border, consider it a mismatch
    if ( x<2 || y<2 || x>=conf.imgsize(2) || y>=conf.imgsize(1) )
      fm(iter) = conf.MAX_R/KL_prev.rho(iter, 2);
      
      if conf.debug && 0
        printf("KL #%4d @ frame #%4d: outside border after reprojection; y: %f, x: %f\n", ...
          iter, KL_prev.frame_id, p_pji_y, p_pji_x);
      end
      
      if(ReWeight)
        fm(iter) *= weight;
      end
      df_dPi(iter, :) = 0;
      DResidualNew(iter) = conf.MAX_R;
      continue;
    end
    
    %temporairly rotate gradient on z axis for improved matching (?)
    %todo: could this be the direction m_n? Seeing eq. 6 and `fi = dot(d, u_m);`, I'm almost certain
    % from the paper: "A search for the closest edge in the new image is then performed
    %                  in the perpendicular direction m_n, up to a distance of max_d pixels"
    kl_m_m_copy = KL_prev.grad(iter, :);
    
    KL_prev.grad(iter,1) = RM(2,1) * kl_m_m_copy(2) + RM(2,2) * kl_m_m_copy(1); % todo: possibly fucked up matrix coordinates
    KL_prev.grad(iter,2) = RM(1,1) * kl_m_m_copy(2) + RM(1,2) * kl_m_m_copy(1);
    
    % Calc_f_J was here
    
    % f_inx corresponds to (y,x)
    kl_iter = KLidx_field(y,x); % index from current frame
    % if not -1, this is the closest (in image) KL to our KL_prev(iter) after it has been warped
    if kl_iter < 0
      df_dPi(iter, :) = 0;
      fm(iter) = conf.MAX_R ./ KL_prev.rho(iter,2);
      fi = 0;
    else
      % Test_f_k was here - a quick test for KL match
      % f_m - gradient from current frame (extraced from aux) - apparently the gradient is not normalized
      % kl - keyline from previous frame
      m1 = KL.grad(kl_iter, :); % current frame non-normalized gradient
      
      m2 = KL_prev.grad(iter, :); %old frame gradient
      
      m2_norm_sq = KL_prev.norm(iter).^2;
      pablo_escobar = dot(m1, m2); 
      
      % from the paper: "For the closest edge found, weak matching is performed by comparing
      %                  the two gradients m_0 and m_n. If the difference is above a certain
      %                  threshold then no match is found (...)
      % | m1.m2 - m2.m2|/m2.m2 > theshold?
      if abs(pablo_escobar - m2_norm_sq) > conf.MATCH_THRESH * m2_norm_sq
        df_dPi(iter, :) = 0;
        fm(iter) = conf.MAX_R ./ KL_prev.rho(iter,2);
        fi = 0;
      else
        d = [p_pji_y p_pji_x] - KL.posSubpix(kl_iter,:); % the distance vector: q_t - q_n
        
        u_m = KL.vers(kl_iter, :);
        
        fi = dot(d, u_m); %residual projected in direction of the gradient
        
        df_dPi(iter, :) = u_m ./ KL_prev.rho(iter,2); % todo: why?
        
        mnum += 1;
        KL_prev.forward(iter) = kl_iter;
        fm(iter) = fi ./ KL_prev.rho(iter, 2); % d_m_i / sigma_rho_i
        
      end  
    end
    
    KL_prev.grad(iter,:) = kl_m_m_copy;
    
    if(ReWeight)
      fm(iter) *= weight;
      df_dPi(iter, :) *= weight;
    end
    
    DResidualNew(iter) = fi;
    
  end
  
  KL_prev_forward = KL_prev.forward; % set the return parameter
  
  if(ProcJF)
    %contrary to comments in C, rho_t is used, not rho_p
    %todo: why why WHY
    Jm = zeros(pnum, 6); % LTCV Jacobian
    RhoTmp0 = zeros(pnum, 1);
    
    RhoTmp0 = conf.zf * PtIm(:, 3); %z coordinate constant mul
    
    %Jx = df_Pix * zf * rho_t
    Jm(:,2) = RhoTmp0 .* df_dPi(:,2);
    %Jy = df_Piy * zf * rho_t
    Jm(:,1) = RhoTmp0 .* df_dPi(:,1);
    %Jz = rho_t * qx_t * df_dPix + rho_t * qy_t * df_dPiy
    %zx
    RhoTmp0 = PtIm(:,3) .* PtIm(:,2);
    Jm(:,3) = RhoTmp0 .* df_dPi(:,2);
    %zy
    RhoTmp0 = PtIm(:,3) .* PtIm(:,1);
    Jm(:,3) += RhoTmp0 .* df_dPi(:,1);
    
    %Jwy = Jy * pz_t
    Jm(:,5) = Jm(:,1) .* Ptm(:,3);
    Jm(:,5) += Jm(:,3) .* Ptm(:,1);
    %Jwx = Jx * pz_t
    Jm(:,4) = Jm(:,2) .* Ptm(:,3);
    Jm(:,4) += Jm(:,3) .* Ptm(:,2);
    %Jwz = -Jx * py_t + Jy * px_t
    RhoTmp0 = Jm(:,2) .*Ptm(:,1);
    Jm(:,6) = -1 * RhoTmp0;
    Jm(:,6) += Jm(:, 1) .* Ptm(:,2);
    
    %there should be a fragment to 0 the non-4-divisible knum, but due to no need for it in octave we'll omit this part
    
    %dot product on half of the JtJ
    % JtJ = Jm' * Jm
    % JtF = Jm' * fm
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
    % doesn't matter, always false
    % and wouldn't make sense otherwise (something seems to be missing)
    % because uncertainties are read here, and they are set to meaningful
    % values only after all TryVelRot calls
  %end
                                
end

%%%%%%%%%%%%%% Minimizer_RV starts here %%%%%%%%%%%%%%

  %% AUXILIARY IMAGE %%
  [distance_field, KLidx_field] = AuxiliaryImage(KL);
  
  % UsePriors is always false
  conf = Config();
  % init_type is always 2
  max_s_rho = EstimateQuantile(KL_prev);
  INIT_ITER = 2; % Actually controls ProcJF in TryVelRot (true or false depending on iteration)
  

  if size(KL_prev.idx , 1) < 1
    if conf.debug
      printf('No keylines @frame #%4d!\n', KL_prev.frame_id)
    end  
    F = 0;
    RVel = eye(3)*1e50;
    RW0 = eye(3)*1e50;
    return
  end

  JtJ = zeros(6,6); % jacobian: J' * J       see Wikipedia
  ApI = zeros(6,6); % [J' * J] + u * I       see Wikipedia
  JtJnew = zeros(6,6);
  
  JtF = zeros(6,1); % estimated residuals: (J' * [y - f(beta)])       see Wikipedia
  JtFnew = zeros(6,1);
  
  h = zeros(6,1); % increment to the estimated parameter vector X
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
  P0Im(:,1) = KL_prev.posImage(:,1); % y
  P0Im(:,2) = KL_prev.posImage(:,2); % x
  P0Im(:,3) = KL_prev.rho(:,1);
  
  %proyect (eq 4.) imgage -> 3d
  P0m(:,3) = 1./ P0Im(:,3); % depth from inverse depth
  Pz_zf = 1./conf.zf * P0m(:,3);
  P0m(:,1) = P0Im(:,1) .* Pz_zf; %y
  P0m(:,2) = P0Im(:,2) .* Pz_zf; %x
  
  F = 0; %energy scores
  Fnew = 0;
  F0 = 0;
  
  v = conf.LM_INIT_V; % u is multiplied by v if there is no gain (step towards gradient descent)
  % in case of consecutive lack of gain, v gets even larger (v *= 2); any successful update sets it back to 2
  
  u = 0; % u seems to be the damping parameter (lambda in Wikipedia notation)
  gain = 0;
  
  eff_steps = 0; %count how many times we gained
  
  %zero init (X is zeros)
  %no reweighting, calculate jacobians
  % input Residual doesn't really matter, because there is no reweighting
  [F, JtJ, JtF, KL_prev.forward, Rest] = TryVelRot(
    0,1,X,Vel, W0, 
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field);
  
  F0 = F; 
  u = conf.TAU * max(max(JtJ)); % see:
  % Madsen, K., Nielsen, N., and Tingleff, O.: 2004, Methods for nonlinear least squares problems.
  % eq. 3.14
  
  % two iterations, first with calculating Jacobians, then without
  %   (because they won't be used - ultimate we need Xnew and Fnew)
  % sill no reweighting
  % if there is any gain, save: score, VelRot, Jacobians, JtF;
  %   and reduce damping parameter (usualy by factor 0.33)
  % final residuals are in Rest
  for iter = 1:INIT_ITER
    % solve [JtJ + u*I]*h=-JtF
    % todo: afaik ApI is positive definite for u > 0, which means that Cholesky could be always used
    ApI = JtJ + eye(6) * u; % todo: use Marquadt refinement: ApI = JtJ + diag(JtJ) * u
    [U, D, Vt] = cv.SVD.Compute(ApI); 
    h = cv.SVD.BackSubst(U, D, Vt, -JtF); % back substitution
   
    Xnew = X+h;
    
    if iter == INIT_ITER
      ProcJF = 0;
    else
      ProcJF = 1;
    end
    
    [Fnew, JtJnew, JtFnew, KL_prev.forward, Rest] = TryVelRot(
      0,ProcJF,X,Vel,W0, 
      KL_prev, KL, P0m,
      max_s_rho,Residual, distance_field, KLidx_field);
      
    if iter == INIT_ITER
      gain = F - Fnew;
    else
      gain = (F-Fnew)/(0.5 * h' * (u*h-JtF)); % see:
      % Madsen, K., Nielsen, N., and Tingleff, O.: 2004, Methods for nonlinear least squares problems.
      % "gain ratio" just below eq. 3.14
    end     
    
    if gain > 0
      F=Fnew;
      X=Xnew;
      JtJ = JtJnew;
      JtF = JtFnew;
      u *= max( 0.33, 1-((2*gain-1)^3)); % todo: this seems dumb (but maybe it's a feature)
      % gain will usually be large, so second expression will be a large negative, so 0.33 will be always selected
      v = 2;
      eff_steps++;
    else
      u *= v;
      v *= 2;
    end
  end
  
  % save zero-init results into temp variables
  Xt = X;
  Ft = F; %score after three iterations
  F0t = F0; %score after first iteration
  ut = u;
  vt = v;
  eff_steps_t = eff_steps;
  
  eff_steps = 0;
  
  %initialization with prior velocity and rotation
  %exactly the same as above, with different X (duh!) and ResidualNew instead of Rest
  X = [Vel; W0]; %usePriors
  [F, JtJ, JtF, KL_prev.forward, ResidualNew] = TryVelRot(
    0,1,X,Vel,W0,
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
    
    [Fnew, JtJnew, JtFnew, KL_prev.forward, ResidualNew] = TryVelRot(
      0,ProcJF,Xnew,Vel,W0,
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
  
  % restore that state that was best
  if F>Ft
    X = Xt;
    F = Ft;
    F0 = F0t;
    u = ut; %u and v will be overwritten in a moment :-(
    v = vt;
    eff_steps = eff_steps_t;
    ResidualNew = Rest;
  end
  
  %swap Residual with ResidualNew
  %Residual contains only zeros at this point
  Residual = ResidualNew;
  ResidualNew = Residual*0;
  
  %reweight
  [F0, JtJ, JtF, KL_prev.forward, ResidualNew] = TryVelRot(
    1,1,X,Vel,W0,
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field);
  F0 = F; 
  u = conf.TAU * max(max(JtJ));
  v = 2;
  
  %todo: why don't we check gain here?
  
  for iter = 1:conf.ITER_MAX
    ApI = JtJ + eye(6) * u;
    
    %todo: Cholesky
    %todo: check out +cv/solve.m
    % Cholesky is faster than SVD, but unstable
    %[R, P, Q] = chol(ApI);
    [U, D, Vt] = cv.SVD.Compute(ApI); 
    h = cv.SVD.BackSubst(U, D, Vt, -JtF); % back substitution
    
    Xnew = X+h;
    [Fnew, JtJnew, JtFnew, KL_prev.forward, ResidualNew] = TryVelRot(
      1,1,Xnew,Vel,W0,
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
      
      % now they really have to be swapped
      tRes = Residual;
      Residual = ResidualNew;
      ResidualNew = tRes;
    else
      u *= v;
      v *= 2;
    end
  end
  
  %todo: Cholesky
  %todo: check out +cv/invert.m
  %[R, P, Q] = chol(JtJ);
  %todo: RRV
  RRV = inv(JtJ); % todo: some sources multiply this matrix by MSE, others not
  % http://www.mathworks.com/help/stats/nlinfit.html#output_argument_d0e530105
  % https://stats.stackexchange.com/questions/231868/relation-between-covariance-matrix-and-jacobian-in-nonlinear-least-squares
  % Numerical Recipes p. 802 eq. 15.5.15
  
  MSE = 1;%(Residual' * Residual) / (KL_prev.ctr - 6);
  
  Vel = X(1:3);
  W0 = X(4:6);
  
  % Setting the uncertainties
  RVel = RRV(1:3, 1:3) * MSE;
  RW0 = RRV(4:6,4:6) * MSE;
  
  if eff_steps>0
    norm_h = h./sqrt(dot(h,h));
    norm_X = X./sqrt(dot(X,X));
    rel_error = norm_h/(norm_X + 1e-30);
    rel_error_score = F/F0;
  else
    rel_error = 1e20;
    rel_error_score = 1e20;
  end
  
  FrameCount++;
  
end
