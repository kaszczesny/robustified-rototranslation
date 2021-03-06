1;
  
function [s_rho] = EstimateQuantile(KL)
  % based on edge_tracker EstimateQuantile
  % tells which s_rho (uncertainty) is such that S_RHO_MIN-s_rho == quantile
  
  
  global conf;
  bins = zeros(conf.N_BINS, 1);

  for iter = 1:KL.ctr
    %check in which bin s_rho is and increment its counter
    idx = floor( conf.N_BINS * (KL.rho(iter,2)-conf.S_RHO_MIN) / (conf.S_RHO_MAX - conf.S_RHO_MIN) ) + 1;
    %todo: idx is nan
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
    if (a > conf.PERCENTILE * KL.ctr )
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
    DResidualNew, ...
    fm, why, escobar ...
] = TryVelRot( ...
    ReWeight, ... % Rewighting switch
    ProcJF, ... % Calculate Jacobians or just energy score?
    ... % UsePriors is always false
    ... % JtJ, ... % Estimated Jacobian Matrix 6x6 - only returned
    ... % JtF, ... % Estimated Residual Matrix 6x1 - only returned
    VelRot, ... % Proposed state vector 6x1 [Translation Rotation]
    ... % V0p, ... % Translation Model 3x1
    ... % PV0, ... % Model uncertainty 3x3 - actually not used
    ... % W0p, ... % Rotation Model 3x1
    ... % PW0, ... % Model uncertainty 3x3 - actually not used
    KL_prev, KL, ... % keylines
    P0m, ... % linear transpose coodinates vector of 3D klist positions (vector or pnum*3)
    ... % pnum, ... % number of points
    max_s_rho, ... % estimated rho 0.9 quantile
    DResidual, ... % Last iteration Distance Residuals - for calculating weights, vector pnumx1
    ... % DResidualNew, ... % New iteration Distance Residuals - new residuals, vector pnumx1
    distance_field, KLidx_field, ...
    FrameCount, ...
    FullScore ... % if 1, score will be a vector before being squared
)
  % returns energy based on dot product of distance residuals

  pnum = KL_prev.ctr;
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
  
  global conf;
  
  % Rt is just flattened R0, Vt is Velocity (1:3) component of VelRot
  
  Ptm = zeros(pnum, 3); %LTCV of transformed 3d coordinates
  PtIm = zeros(pnum, 3); %LTCV of transformed image coordinates
  
  fm = zeros(pnum, 1); %weighted residuals
  df_dPi = zeros(pnum, 2); %weighted img derivative of residual
  
  % perform SE3: Pos = Rot*Pos0 + Vel     (eq. 5 an 2)
  Ptm = (R0 * P0m')'; %todo: nan was here
  for iter=1:3
    Ptm(:, iter) += VelRot(iter);
  end
  
  % equation 3: 3d -> image
  % from the paper: "Points from the previous edge-map (q_0, rho_0) are projected
  %                  into the points (q_t, rho_t) by using the warping function tau.
  PtIm(:,3) = 1 ./ Ptm(:,3);
  Pz_zf = conf.zf * PtIm(:,3);
  PtIm(:,2) = Ptm(:,2) .* Pz_zf; %y
  PtIm(:,1) = Ptm(:,1) .* Pz_zf; %x
  
  PtIm(:,1:2) = normalizedToPixel(PtIm(:,1:2));
  
  % the klist argument in C is actually OLD keymap,
  % while this->auxiliary field holds indexes for CURRENT keymap
  
  n_match_tryvelrot = 0;
  
  why = zeros(conf.imgsize);
  escobar = why;
  
  for iter = 1:pnum
    KL_prev.forward(iter) = -1; % reset forward match
    
    why(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 0;
    
    % don't use this keyline if uncertainty is too high
    % or if  keyline hasn't appeared in at least 2 consecutive frames (todo: investigate how that works)
    % (exluding init)
    if KL_prev.rho(iter,2) > max_s_rho || ...
       KL_prev.frames(iter) < min(conf.MATCH_NUM_THRESH, FrameCount)
       %todo: check if KL_prev.rho(iter,2) == 0

      if KL_prev.rho(iter,2) > max_s_rho
        why(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 1;
      else
        why(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 2;
      end
       
       
      if conf.debug_minimizer
        if KL_prev.rho(iter,2) > max_s_rho
          printf('KL #%4d @frame #%4d: rho uncertainty too high: %f (%f)\n', ...
            iter, KL_prev.frame_id, KL_prev.rho(iter,2), max_s_rho)
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
    p_pji_y = PtIm(iter, 2);
    p_pji_x = PtIm(iter, 1);
    
    x = round(p_pji_x);
    y = round(p_pji_y);
    
    %estimate reweighting
    weight = 1;
    if ReWeight && abs(DResidual(iter)) > conf.REWEIGHT_DISTANCE
      weight = conf.REWEIGHT_DISTANCE ./ abs(DResidual(iter)); % Hubber norm
      % it isn't squared, because fm is squared with a dot product at the end of the function
    end
    
    %if outside border, consider it a mismatch
    if ( x<2 || y<2 || x>=conf.imgsize(1) || y>=conf.imgsize(2) )
      fm(iter) = conf.MAX_R/KL_prev.rho(iter, 2);
      
      why(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 3;
      
      if conf.debug_minimizer >= 2
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
    
    KL_prev.grad(iter,1) = RM(1,1) * kl_m_m_copy(1) + RM(1,2) * kl_m_m_copy(2); % todo: possibly fucked up matrix coordinates
    KL_prev.grad(iter,2) = RM(2,1) * kl_m_m_copy(1) + RM(2,2) * kl_m_m_copy(2);
    
    % Calc_f_J was here
    
    % f_inx corresponds to (y,x)
    kl_iter = KLidx_field(x,y); % index from current frame
    % if not -1, this is the closest (in image) KL to our KL_prev(iter) after it has been warped
    if kl_iter < 0
      df_dPi(iter, :) = 0;
      fm(iter) = conf.MAX_R ./ KL_prev.rho(iter,2);
      fi = 0;
      why(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 4;
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
      escobar(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = abs(pablo_escobar - m2_norm_sq) / m2_norm_sq;
      if abs(pablo_escobar - m2_norm_sq) > conf.MATCH_THRESH * m2_norm_sq
        df_dPi(iter, :) = 0;
        fm(iter) = conf.MAX_R ./ KL_prev.rho(iter,2);
        fi = 0;
        why(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 5;
      else
        d = [p_pji_x p_pji_y] - KL.posSubpix(kl_iter,:); % the distance vector: q_t - q_n
        
        u_m = KL.vers(kl_iter, :);
        
        fi = dot(d, u_m); %residual projected in direction of the gradient
        
        df_dPi(iter, :) = u_m ./ KL_prev.rho(iter,2); % todo: why?
        
        mnum += 1;
        KL_prev.forward(iter) = kl_iter;
        fm(iter) = fi ./ KL_prev.rho(iter, 2); % d_m_i / sigma_rho_i
        
        n_match_tryvelrot++;
        
        why(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 6;
      end  
    end
    
    KL_prev.grad(iter,:) = kl_m_m_copy;
    
    if(ReWeight)
      fm(iter) *= weight;
      df_dPi(iter, :) *= weight;
    end
    
    DResidualNew(iter) = fi;
    
  end
  
  %{
  if conf.visualize_minimizer_insides
    figure(6);
    imagesc(why'); axis equal; colormap jet; colorbar
    title('minimizer why')
    
    figure(7);
    imagesc(escobar'); axis equal; colormap jet; colorbar;
    title('abs(pablo escobar)')
  end  
  %}
  
  n_match_tryvelrot; % suppresing match number
  
  KL_prev_forward = KL_prev.forward; % set the return parameter
  
  if(ProcJF)
    %contrary to comments in C, rho_t is used, not rho_p
    %todo: why why WHY
    Jm = zeros(pnum, 6); % LTCV Jacobian
    RhoTmp0 = zeros(pnum, 1);
    
    RhoTmp0 = conf.zf * PtIm(:, 3); %z coordinate constant mul
    
    %Jx = df_Pix * zf * rho_t
    Jm(:,1) = RhoTmp0 .* df_dPi(:,1);
    %Jy = df_Piy * zf * rho_t
    Jm(:,2) = RhoTmp0 .* df_dPi(:,2);
    %Jz = rho_t * qx_t * df_dPix + rho_t * qy_t * df_dPiy
    %zx
    RhoTmp0 = PtIm(:,3) .* PtIm(:,1);
    Jm(:,3) = RhoTmp0 .* df_dPi(:,1);
    %zy
    RhoTmp0 = PtIm(:,3) .* PtIm(:,2);
    Jm(:,3) += RhoTmp0 .* df_dPi(:,2);
    
    %Jwy = Jy * pz_t
    Jm(:,4) = Jm(:,2) .* Ptm(:,3);
    Jm(:,4) += Jm(:,3) .* Ptm(:,2);
    %Jwx = Jx * pz_t
    Jm(:,5) = Jm(:,1) .* Ptm(:,3);
    Jm(:,5) += Jm(:,3) .* Ptm(:,1);
    %Jwz = -Jx * py_t + Jy * px_t
    RhoTmp0 = Jm(:,1) .*Ptm(:,2);
    Jm(:,6) = -1 * RhoTmp0;
    Jm(:,6) += Jm(:, 2) .* Ptm(:,1);
    
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
  minimizer_residual = DResidualNew( ...
    (DResidualNew ~= conf.MAX_R) & (DResidualNew ~= 0) );
  minimizer_residual_mean = mean(abs(minimizer_residual))

  if FullScore
    score = fm;
  end
 
  %if(UsePriors)
    % doesn't matter, always false
    % and wouldn't make sense otherwise (something seems to be missing)
    % because uncertainties are read here, and they are set to meaningful
    % values only after all TryVelRot calls
  %end
                                
end

%%%%%%%%%%%%%% Minimizer_RV starts here %%%%%%%%%%%%%%
function [ ...
  F, ...
  Vel, W0, RVel, RW0, ...
  KL_prev, ...
  rel_error, rel_error_score, ...
  FrameCount ...
] = MinimizeRV (...
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

  %% AUXILIARY IMAGE %%
  [distance_field, KLidx_field] = AuxiliaryImage(KL);
  
  % UsePriors is always false
  global conf;
  % init_type is always 2
  max_s_rho = EstimateQuantile(KL_prev)
  INIT_ITER = 2; % Actually controls ProcJF in TryVelRot (true or false depending on iteration)
  

  if KL_prev.ctr < 1
    if conf.debug_minimizer
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
  
  pnum = KL_prev.ctr;
  
  P0Im = zeros(pnum, 3); %image coordinates
  P0m = zeros(pnum, 3); %3d coordinates
  
  Residual = zeros(pnum, 1); %Res0;distance residuals
  ResidualNew = zeros(pnum, 1);  %Res1;
  Rest = zeros(pnum, 1);
  
  %converto to ltcv
  P0Im(:,1) = KL_prev.posImage(:,1); % x
  P0Im(:,2) = KL_prev.posImage(:,2); % y
  P0Im(:,3) = KL_prev.rho(:,1);
  
  %proyect (eq 4.) imgage -> 3d
  P0m(:,3) = 1./ P0Im(:,3); % depth from inverse depth
  Pz_zf = 1./conf.zf * P0m(:,3);
  P0m(:,2) = P0Im(:,2) .* Pz_zf; %y
  P0m(:,1) = P0Im(:,1) .* Pz_zf; %x
  
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
    0,1,X, 
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field, FrameCount,0);
  
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
  
  if conf.visualize_score
    score_vec(1,:) = [F,0];
  end
  for iter = 1:INIT_ITER
    % solve [JtJ + u*I]*h=-JtF
    % todo: afaik ApI is positive definite for u > 0, which means that Cholesky could be always used
    ApI = JtJ + eye(6) * u; % todo: use Marquadt refinement: ApI = JtJ + diag(JtJ) * u
    [U, D, Vt] = cv.SVD.Compute(ApI); 
    h = cv.SVD.BackSubst(U, D, Vt, -JtF); % back substitution
   
   
    % visualization of cost function in the direction of Jacobian
    % this is too brutal for program execution to even be in Config
    if 0
      if iter==1
        scores = [];
        h = h ./ sqrt(dot(h,h));
        k=[-0.01:0.0001:0.01];
        for kk=1:length(k)
          Xnew = X + (h*k(kk));
          Fnew = TryVelRot(0,1,Xnew,KL_prev,KL,P0m,max_s_rho,
          Residual,distance_field,KLidx_field, FrameCount, 0);
          scores = [scores Fnew];
        end
        plot(k, scores);
        keyboard("<<<")
      end
    end
   
   
    Xnew = X+h;
    
    if iter == INIT_ITER
      ProcJF = 0;
    else
      ProcJF = 1;
    end
    
    [Fnew, JtJnew, JtFnew, KL_prev.forward, Rest] = TryVelRot(
      0,ProcJF,Xnew,
      KL_prev, KL, P0m,
      max_s_rho,Residual, distance_field, KLidx_field, FrameCount, 0);
      
    if iter == INIT_ITER
      gain = F - Fnew;
    else
      %todo: division by zero was here
      gain = (F-Fnew)/(0.5 * h' * (u*h-JtF)); % see:
      % Madsen, K., Nielsen, N., and Tingleff, O.: 2004, Methods for nonlinear least squares problems.
      % "gain ratio" just below eq. 3.14
    end     
    
    if conf.visualize_score
      score_vec(end+1,:)  = [Fnew,1];
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
    0,1,X,
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field, FrameCount, 0);
  F0 = F; 
  u = conf.TAU * max(max(JtJ));
  v = 2;
  
  if conf.visualize_score
    score_vec(end+1,:)  = [F,2];
  end
  
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
      0,ProcJF,Xnew,
      KL_prev, KL, P0m,
      max_s_rho,Residual, distance_field, KLidx_field, FrameCount, 0);
      
    if iter == INIT_ITER
      gain = F - Fnew;
    else
      gain = (F-Fnew)/(0.5 * h' * (u*h-JtF));
    end  
    
    if conf.visualize_score
      score_vec(end+1,:)  = [Fnew,3];
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
  
  if conf.visualize_score
    score_vec(end+1,:)  = [F,4];
  end
  
  %swap Residual with ResidualNew
  %Residual contains only zeros at this point
  Residual = ResidualNew;
  ResidualNew = Residual*0;
  
  %reweight
  [F, JtJ, JtF, KL_prev.forward, ResidualNew, fm, why, escobar] = TryVelRot(
    1,1,X,
    KL_prev, KL, P0m,
    max_s_rho,Residual, distance_field, KLidx_field, FrameCount, 0);
  F0 = F;
  u = conf.TAU * max(max(JtJ));
  v = 2;
  
  if conf.visualize_score
    score_vec(end+1,:)  = [F,5];
  end
  
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
    [Fnew, JtJnew, JtFnew, KL_prev.forward, ResidualNew, ...
      fmNew, whyNew, escobarNew] = TryVelRot(
      1,1,Xnew,
      KL_prev, KL, P0m,
      max_s_rho,Residual, distance_field, KLidx_field, FrameCount, 0);
    
    gain = (F-Fnew)/(0.5*h'*(u*h-JtF));
    if gain > 0
      F=Fnew;
      X=Xnew;
      JtJ = JtJnew;
      JtF = JtFnew;
      fm = fmNew;
      why = whyNew;
      escobar = escobarNew;
      u *= max( 0.33, 1-((2*gain-1)^3));
      v = 2;
      eff_steps++;
      if conf.visualize_score
        score_vec(end+1,:)  = [F,6];
      end
      
      % now they really have to be swapped
      tRes = Residual;
      Residual = ResidualNew;
      ResidualNew = tRes;
    else
      u *= v;
      v *= 2;
    end
  end
  
  
    im = zeros(conf.imgsize);
    for iter = 1:KL_prev.ctr
      im( KL_prev.pos(iter, 1), KL_prev.pos(iter, 2) ) = Residual(iter);
    end
    
    im_fm = zeros(conf.imgsize);
    for iter = 1:KL_prev.ctr
      im_fm( KL_prev.pos(iter, 1), KL_prev.pos(iter, 2) ) = fm(iter);
    end
    
    save_img(besos(why, 0, 6), KL_prev.frame_id, 6);
    save_img(besos(escobar, 0, 2), KL_prev.frame_id, 7);
    save_img(besos(im, -1.5, 1.5), KL_prev.frame_id, 8);
    save_img(besos(im_fm, -1.5, 1.5), KL_prev.frame_id, 41);
    
  if conf.visualize_minimizer_insides
    figure(6);
    imagesc(why'); axis equal; colormap jet; colorbar
    title('minimizer why')
    
    figure(7);
    imagesc(escobar'); axis equal; colormap jet; colorbar;
    title('abs(pablo escobar)')
  
    figure(8)
    imagesc(im');
    axis equal; colormap jet; colorbar;
    title('residual')
    
    figure(40)
    subplot(2,1,1)
    hist(fm,100)
    subplot(2,1,2)
    hist(Residual,100)
    
    figure(41)
    imagesc(im_fm')
    axis equal; colorbar; colormap jet;
  end  
  
  %todo: Cholesky
  %todo: check out +cv/invert.m
  %[R, P, Q] = chol(JtJ);
  %todo: RRV
  if cond(JtJ) > 1e10
    % avoid inverting singular matrices
    printf("JtJ is singular\n");
    RRV = zeros(6) + conf.S_RHO_INIT;
  else  
    RRV = inv(JtJ);
  end  
  % todo: some sources multiply this matrix by MSE, others not
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
  
  if conf.visualize_score
    figure(9)
    colors = [0,0,0;
              0,0,1;
              0,1,0;
              0,1,1;
              1,0,0;
              1,0,1;
              1,1,0;
              1,1,1];
    plot(score_vec(:,1),"k")
    hold on;
    for iter = 1:size(score_vec,1)
      plot(iter,score_vec(iter,1),"*o", 'MarkerSize', 20, 'Color', colors(score_vec(iter,2)+1,:));
    end
    hold off;
    title('minimizer score')
  end
  
  FrameCount++;
  
  if cond(JtJ) > 1e10
    Vel = NaN(3,1);
    W0 = NaN(3,1);
  end
  
end
