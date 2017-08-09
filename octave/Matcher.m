1;

function [KL] = ForwardRotate( KL, R )
  % forward rotate old KL points
  zf = Config().zf;
  
  for iter=1:KL.ctr
    q = R * [KL.posSubpix(iter,:)./zf, 1]';
    
    if abs(q(3)) > 0
      KL.posSubpix(iter,:) = q(1:2) ./q(3) * zf;
      KL.rho(iter,:) /= q(3);
    end
  end
  
  % nasty rotation (?)
  q = R * [KL.grad(iter,:), 0]';
  KL.grad(iter,:) = q(1:2);
end

function [nmatch, KL] = ForwardMatch(KL, KL_prev)
  nmatch = 0;
  for iter = 1:KL_prev.ctr
    ikl_f = KL_prev.forward(iter);
    if ikl_f < 0 || ikl_f < KL.ctr
      continue
    end
  
    % clone to new
    KL.rho(ikl_f,:) = KL_prev.rho(iter,:);
    KL.frames(ikl_f) = KL_prev.frames(iter) + 1;
    KL.matching(ikl_f) = iter;
    KL.posImageMatch(ikl_f,:) = KL_prev.posImage(iter,:);
    KL.matchedGrad(ikl_f,:) = KL_prev.grad(iter,:);
    KL.matchedNorm(ikl_f) = KL_prev.norm(iter);
    
    nmatch++;
  end
end

function [nmatch, KL] = DirectedMatching(...
  Vel, RVel, R, KL_prev, KL_prev_img_mask, KL) %and constants
  
  nmatch = 0;
  
  Vel = R * Vel; %back rotate translation
  RVel = R * RVel * R';
  
  for iter = 1:KL.ctr
    % clear is always false
    
    i_mch = SearchMatch( KL_prev, KL_prev_img_mask, KL, iter, ...
      Vel, RVel, R );
      
    if i_mch < 0
      continue
    end
    
    % clone data from previous keylines
    KL.rho(iter,:) = KL_prev.rho(i_mch,:);
    KL.matching(iter) = i_mch;
    KL.frames(iter) = KL_prev.frames(i_mch) + 1;
    KL.posImageMatch(iter,:) = KL_prev.posImage(i_mch,:);
    KL.matchedGrad(iter,:) = KL_prev.grad(i_mch,:);
    KL.matchedNorm(iter) = KL_prev.norm(i_mch);
    
    nmatch++;
    
  end
  
end

function [idx] = SearchMatch( KL_prev, KL_prev_img_mask, KL, ...
  k, ... % index of KL in new frame
  Vel, RVel, R) % R is back rotation
  %called by DirectedMatching
  
  conf = Config();
  zf = Config().zf;

  dq_min = 0;
  dq_max = 0;
  t = [0 0];
  dq_rho = 0;
  
  t_steps = 0;
  p_m = [0 0]; %backrotated keypoint projected to image coords
  k_rho = 0; %backrotated inverse depth
  
  p_m3 = R * [KL.posImage(k,:)'; zf]; % backrotate the new keypoint to look on the old edge map's mask
  
  p_m = p_m3(1:2) * zf ./ p_m3(3);
  k_rho = KL.rho(k,1) * zf ./ p_m3(3);
  
  pi0 = p_m + conf.principal_point';
  
  t = -(Vel(1:2) * zf - Vel(3) * p_m);
  
  norm_t = norm(t);
  
  DrDv = [zf, zf, -p_m(2) - p_m(1)];
  sigma2_t = DrDv * RVel * DrDv'; % estimated uncertainty in the displacement

  if conf.debug_matching
    if norm_t == 0
      printf('KL #%4d @frame #%4d: displacement is exactly zero\n', ...
            iter, KL_prev.frame_id)
    end
  end
  
  % defining direction and area of search
  if norm_t > 1e-6 %|| norm_t == 0
    if norm_t > 0
    t /= norm_t;
    end
    
    % Prior for the translation distance, only useful if forward matching exist
    dq_rho = k_rho * norm_t;
    
    % Minimum distance to seach for, constrained by -ERR_DQ
    dq_min = max(0, norm_t*(k_rho - KL.rho(k,2))) - conf.LOCATION_UNCERTAINTY_MATCH;
    
    % Maximum distance to seach for, constrained by max_radius+ERR_DQ
    dq_max = min(conf.SEARCH_RANGE, norm_t*(k_rho + KL.rho(k,2))) + conf.LOCATION_UNCERTAINTY_MATCH;
    
    if dq_rho > dq_max % If stating point grater than max
      dq_rho = (dq_max + dq_min)/2; % Start from the midle in both direction
      
      t_steps = round(dq_rho);
    else
      % Seach start from dq_rho to both dicerctions, the
      % number of iterations is the maximun
      t_steps = round(max(dq_max-dq_rho, dq_rho-dq_min));  
    end
  else
      % If no displacemt (not common) search in perpendicular direction of the edge
      t = KL.grad(k,:);
      norm_t = KL.norm(k);
      t /= norm_t;
      norm_t = 1;
      
      dq_min = -conf.SEARCH_RANGE - conf.LOCATION_UNCERTAINTY_MATCH;
      dq_max =  conf.SEARCH_RANGE + conf.LOCATION_UNCERTAINTY_MATCH;
      
      dq_rho = 0;
      t_steps = dq_max;
      
      if conf.debug_matching
        printf('KL #%4d @frame #%4d: no displacement, halfline in gradient\n', ...
              k, KL.frame_id)
      end
  end
  
  norm_m = KL.norm(k);
  
  %displacement counters
  tn = dq_rho;
  tp = dq_rho + 1;
  
  for t_i = 1:t_steps
    for i_inx = 0:1
      if i_inx
        tt = tp;
        if tt > dq_max
          continue
        end
      else  
        tt = tn;
        if tt < dq_min
          continue
        end
      end
      
      inx_y = round(t(2) + pi0(2));
      inx_x = round(t(1) + pi0(1));
      if inx_y < 1 || inx_y > size(KL_prev_img_mask,2) || inx_x < 1 || inx_x > size(KL_prev_img_mask,1)
        continue
      end  
      j = KL_prev_img_mask( inx_x, inx_y );
      if j == 0
        continue
      end
    
      norm_m0 = KL_prev.norm(j);
      
      cang = dot(KL_prev.grad(j,:), KL.grad(k,:)) / (norm_m0*norm_m);
      
      if cang < conf.MATCH_THRESH_ANGLE_COS || ...
         abs(norm_m0/norm_m - 1) > conf.MATCH_THRESH_MODULE
         
         if conf.debug_matching && 0
          if cang < conf.MATCH_THRESH_ANGLE_COS
            printf('KL #%4d @frame #%4d: angle threshold with %4d: %f\n', ...
                  k, KL.frame_id, j, cang)
          else
            printf('KL #%4d @frame #%4d: modulus threshold with %4d: %f\n', ...
                  k, KL.frame_id, j, abs(norm_m0/norm_m - 1))
          end  
        end
         continue
      end
      
      rho = KL_prev.rho(j,1);
      s_rho = KL_prev.rho(j,2);
      
      v_rho_dr = conf.LOCATION_UNCERTAINTY_MATCH.^2 + ...
        s_rho.^2 * norm_t.^2 + ...
        sigma2_t * rho.^2; %whaaat the fak
        
      if (tt - norm_t *rho).^2 > v_rho_dr
        if conf.debug_matching
          printf('KL #%4d @frame #%4d: model inconsistent\n', ...
                  iter, KL_prev.frame_id)
        end
        continue
      end  
        
      idx = j;
      return  
      
    end
    
    tp++;
    tn--;
  end
  
  idx = -1;
  
end

function [r_num, KL] = Regularize1Iter(KL)
  % performs regularization of depth and depth uncertainty
  

  % from paper: "The underlying idea in this regularization step is
  %              that close points in an image are likely to be close
  %              in space. (...) In order to add some resilience to
  %              this systematic error, neighbouring keylines are
  %              double tested before performing regularization."
  
  thresh = Config().REGULARIZE_THRESH; % angular threshold
  % estimated uncertainty is the cut-off
  
  % todo: thresh is cos(beta). Beta should be 45, not 60
  
  r_num = 0;
  
  r = zeros(KL.ctr,1);
  s = zeros(KL.ctr,1);
  
  mask = zeros(KL.ctr,1); % named "mask" because "set" is a keyword
  
  for iter=1:KL.ctr
    if (KL.idx(iter,1) == 0 || KL.idx(iter,2) == 0)
      continue %no neighbors
    end
    
    kn = KL.idx(iter,2); % next KL
    kp = KL.idx(iter,1); % previous KL
    
    rho = KL.rho(iter,1);
    rho_p = KL.rho(kp,1);
    rho_n = KL.rho(kn,1);
    
    % sigma is generally s_rho.^2
    sigma = KL.rho(iter,2)^2;
    sigma_p = KL.rho(kp,2)^2;
    sigma_n = KL.rho(kn,2)^2;
    
    if (rho_n - rho_p).^2 > sigma_n + sigma_p
      % todo: in article it's a bit different (eq 15):
      %  abs(rho_n - rho_p) > sigma_n + sigma_p
      %  ^ this was squared    ^ this wasn't 
      continue %uncertainty test (probabilistic)
    end
    
    alpha = dot(KL.grad(kn,:), KL.grad(kp,:)) ./ ...
      (KL.norm(kn) * KL.norm(kp));
    
    if alpha - thresh < 0
      % regularization is only performed if final alpha is > 0
      continue
    end
    
    alpha = (alpha-thresh)/(1-thresh); % weighting factor: [0, 1]
    
    %if sigma == 0 || sigma_n == 0 || sigma_p == 0
      % todo: not sure why this happens
    %  continue
    %end
    
    wr = 1/sigma;
    wrn = alpha/sigma_n;
    wrp = alpha/sigma_p;
    
    r(iter) = dot([wrn wr wrp], [rho_n rho rho_p]) / ...
      sum([wrn wr wrp]);
    % todo: have used s_rho squared
    s(iter) = dot([wrn wr wrp], sqrt([sigma_n sigma sigma_p])) / ...
      sum([wrn wr wrp]);
      
    mask(iter) = 1;
    r_num++;    
  end
  for (iter=1:KL.ctr)
    if(mask(iter))
      KL.rho(iter,1) = r(iter);
      KL.rho(iter,2) = s(iter);
    end
  end
  
  return
end

function [KL] = UpdateInverseDepthKalman(...
 Vel, RVel, RW0, KL) % 1e-5
 
  %{
    Mapping between rebvo and OpenCV Kalman
    
    Correct <-- Y
    Predict --> p_p
    
    controlMatrix --> 0
    errorCovPost --> v_rho ((1-K*H) * p_p)?
    errorCovPre --> v_rho (KL.s_rho^2)
    gain --> K
    measurementMatrix --> H
    measurementNoiseCov --> Mk * R * Mk'
    processNoiseCov --> L * Q * L'
    state_post --> rho_p
    state_pre --> KL.rho
    transitionMatrix --> F
  %}
  
  
 
  % todo: use openCV kalman (or maybe not)
  
  conf = Config();
  zf = conf.zf;
 
  for (iter = 1:KL.ctr)
    if KL.matching(iter) < 0
      continue
    end  
    
    % debug cout panic
    KL.rhoPredict(iter,2) = KL.rho(iter,2);
    
    % keyline new homo coordinates
    q = KL.posImage(iter,:);
    
    % keyline old homo coordinates
    q0 = KL.posImageMatch(iter,:);
    
    % previous inverse depth variance: P(k-1)
    v_rho = KL.rho(iter,2)^2;
    
    % shortcut for edge perpendicular direction
    u = KL.matchedGrad(iter,:) ./ KL.matchedNorm(iter);
    
    % pixel displacement projected on u (the noised observation): z(k)
    % left side of eq. 22
    Y = dot(u, (q-q0));
    
    % right side of eq. 23, excluding rho_p
    % "observation model which maps the true state space into the observed space"
    % h(x(k), v) = H * x(k) - N(0,1)
    H = Vel(1:2)'*zf - q0*Vel(3);
    H = dot(u, H);
    
    %%% PREDICTION %%%
    
    % inverse depth prediction: x(k)
    % prediction equation is NOT eq. 21, but:
    % f(x(k-1), w) = [1/(1/x(k-1) + Vel_z) * N(1,Q_Rel)] + N(0,Q_Abs)
    rho_p = 1 / ( 1/KL.rho(iter,1) + Vel(3) );
    KL.rhoPredict(iter,1) = rho_p;
    
    % transition between previous and current state
    % F = df/dx(k-1)
    F = 1 / (1+ KL.rho(iter,1) * Vel(3)); %jacobian strikes back
    F *= F;
    
    % covariance prediction: P(k) = F * P(k-1) * F' + L * Q * L';
    % L = [df/dN1, df/dVel_z, df/dN2] = [x(k-1), -x(k).^2, 1]
    % Q = [sigma_N1, sigma_Vel_z, sigma_N2] = [Q_Rel^2, sigma_Vel_z, Q_Abs^2]
    p_p = F*v_rho*F + ... % uncertainty propagation
          (KL.rho(iter,1)*conf.RESHAPE_Q_RELATIVE)^2 + ... % relative uncertainty model
          rho_p^2 * RVel(3,3) * rho_p^2 + ... % uncertainty on the Z velocity
          conf.RESHAPE_Q_ABSOLUTE^2; % absolute uncertainty model
    
    %%% UPDATE %%%
    
    % measurement pre-fit residual: y
    e = Y-H*rho_p; % error correction
    
    %partial derivative of the correction equation with respect to uncertainty sources
    Mk = [ -1 , % df/dw
          u(1)*rho_p*zf, % df/dVel_x
          u(2)*rho_p*zf, % df/dVel_y
          -rho_p * dot(u, q0), % df/dVel_z
          u(1)*rho_p*Vel(3), % df/dq_ox
          u(2)*rho_p*Vel(3) ]'; % df/dq_oy
    
    R = zeros(6,6);
    loc_unc_sq = conf.LOCATION_UNCERTAINTY^2;
    R(1,1) = loc_unc_sq; % sigma_w
    R(2:4,2:4) = RVel; % sigma_Vel_x, sigma_Vel_y, sigma_Vel_z (this probably should be just the diagonal)
    R(5,5) = loc_unc_sq; % sigma_q_ox
    R(6,6) = loc_unc_sq; % sigma_q_oy
    
    %Kalman update equations
    
    % pre-fit residual covariance
    S = H*p_p*H + (Mk*R*Mk');
    
    % Kalman gain
    K = p_p*H*(1/S);
    
    % updated state estimate
    KL.rho(iter,1) = rho_p + K*e;
    
    % updated state estimate covariance
    v_rho = (1-K*H) * p_p;
    if v_rho < 0 % equivalent to Nan, but Octave returns complex number
      KL.rho(iter,1) = conf.RHO_INIT;
      KL.rho(iter,2) = conf.S_RHO_MAX;
      continue
    end
    
    % standard deviation
    KL.rho(iter,2) = sqrt(v_rho);
    
    % is inverse depth goes beyond limit, apply correction
    if(KL.rho(iter,1) < conf.S_RHO_MIN)
      KL.rho(iter,2) += conf.S_RHO_MIN - KL.rho(iter,1);
      KL.rho(iter,1) = conf.S_RHO_MIN;
    elseif KL.rho(iter,1) > conf.S_RHO_MAX
      KL.rho(iter,1) = conf.S_RHO_MAX;
    elseif sum(isnan(KL.rho(iter,:))) > 0 || ...
            sum(isinf(KL.rho(iter,:))) > 0
      KL.rho(iter,1) = conf.RHO_INIT;
      KL.rho(iter,2) = conf.S_RHO_MAX;
    elseif KL.rho(iter,2) < 0 % s_rho == 0 is no good either
      %todo: why does it appear, then?
      KL.rho(iter,1) = conf.RHO_INIT;
      KL.rho(iter,2) = conf.S_RHO_MAX;
    end
  
  end
 
end

function [KL, ...
  Kp, ... %estimated ratio: sqrt(k)
  P_Kp ... % Estimated uncertainty on global depth
] = EstimateReScaling(KL)
  
  s_rho_min = Config().S_RHO_MAX; %because f you, that's why
  MATCH_NUM_MIN = Config().MATCH_NUM_MIN; % always 1
  
  if KL.ctr <= 0
    Kp = 1;
    return
  end

  rTr = 0;
  rTr0 = 0;
  
  for iter=1:KL.ctr
    if KL.frames(iter) < MATCH_NUM_MIN || ...
      KL.rhoPredict(iter,2) <= 0 || ...
      KL.rho(iter,2) > s_rho_min %|| ...
      %KL.rhoPredict(iter,2) == 0 %todo: temporary test
        continue
    end
    
    rTr  += (KL.rho(       iter,1) / KL.rhoPredict(iter,2)).^2;
    rTr0 += (KL.rhoPredict(iter,1) / KL.rhoPredict(iter,2)).^2;
    %nan check -> printf
  end
  
  if rTr0 > 0
    Kp = sqrt(rTr/rTr0); % sqrt(k)
  else
    Kp = 1;
  end

  % this value isn't really used anywhere, so we simply avoid div0 warning
  %if rTr == 0
    P_Kp = inf;
  %else  
  %  P_Kp = 1./rTr;
  %end  

  % re_escale is always true
  KL.rho /= Kp;
  
end

