1;

function [KL] = ForwardRotate( KL, R )
  % forward rotate old KL points
  conf = Config();
  
  q = [ KL.posImage'./conf.zf; ones(1,KL.ctr) ];
  q = (R * q)';
  
  q3 = q(:, 3);
  q33 = repmat(q3, 1, 2);
  q12 = q(:, 1:2) ./ q33;
  
  q = q ./ repmat(q(:,3), 1, 3);
  idx = abs(q33) > 0;
  KL.posImage(idx) = q12(idx) * conf.zf;
  %todo rho not finished!
  
  % nasty rotation (?)
  q = [ KL.grad'; zeros(1,KL.ctr) ];
  q = (R * q)';
  KL.grad = q(:,1:2);
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
    KL.matching = i_mch;
    KL.frames(iter) = KL_prev.frames(i_mch) + 1;
    KL.posImageMatch(iter,:) = KL_prev.posImage(i_mch,:);
    KL.matchedGrad(iter,:) = KL_prev.grad(i_mch,:);
    
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
  
  pi0 = p_m + conf.principal_point;
  
  t = -(Vel(1:2) * zf - Vel(3) * p_m);
  
  norm_t = sqrt(dot(t,t));
  
  DrDv = [zf, zf, -p_m(2) - p_m(1)];
  sigma2_t = DrDv * RVel * DrDv'; % estimated uncertainty in the displacement
  
  % defining direction and area of search
  if norm_t > 1e-6
    t /= norm_t;
    
    % Prior for the translation distance, only useful if forward matching exist
    dq_rho = k_rho * norm_t;
    
    % Minimum distance to seach for, constrained by -ERR_DQ
    dq_min = max(0, norm_t*(k_rho - KL.rho(k,1))) - conf.LOCATION_UNCERTAINTY_MATCH;
    
    % Maximum distance to seach for, constrained by max_radius+ERR_DQ
    dq_max = min(conf.SEARCH_RANGE, norm_t*(k_rho + KL.rho(k,1))) + conf.LOCATION_UNCERTAINTY_MATCH;
    
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
      t = KL.grad(iter,:);
      norm_t = sqrt(dot(t,t));
      t /= norm_t;
      norm_t = 1;
      
      dq_min = -conf.SEARCH_RANGE - conf.LOCATION_UNCERTAINTY_MATCH;
      dq_min =  conf.SEARCH_RANGE + conf.LOCATION_UNCERTAINTY_MATCH;
      
      dq_rho = 0;
      t_steps = dq_max;
  end
  
  norm_m = KL.grad(k,:);
  norm_m = sqrt(dot(norm_m, norm_m));
  
  %displacement counters
  tn = dq_rho;
  tp = dq_rho + 1;
  
  for t_i = 1:t_steps
    for i_inx = 1:2
      if i_inx == 1
        tt = tn;
        if tt < dq_min
          continue
        end
      else  
        tt = tp;
        if tt > dq_max
          continue
        end
      end
      
      inx_y = round(t(1) + pi0(1));
      inx_x = round(t(2) + pi0(2));
      if inx_y < 1 || inx_y > size(KL_prev_img_mask,1) || inx_x < 1 || inx_x > size(KL_prev_img_mask,2)
        continue
      end  
      j = KL_prev_img_mask( inx_y, inx_x );
      if j == 0
        continue
      end
    
      m_m = KL_prev.grad(j,:);;
      norm_m0 = sqrt(dot(m_m, m_m));
      
      cang = dot(m_m, KL.grad(k,:)) / (norm_m0*norm_m);
      
      if cang < conf.MATCH_THRESH_ANGLE_COS || ...
         abs(norm_m0/norm_m - 1) > conf.MATCH_NUM_MIN
         
         continue
      end
      
      rho = KL.rho(k,1);
      s_rho = KL.rho(k,2);
      
      v_rho_dr = conf.LOCATION_UNCERTAINTY_MATCH.^2 + ...
        s_rho.^2 * norm_t.^2 + ...
        sigma2_t * rho.^2;
        
      if (tt - norm_t *rho).^2 > v_rho_dr
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
  
  %thresh = conf.????
  
  r_num = 0;
  
  r = zeros(KL.ctr,1);
  s = zeros(KL.ctr,1);
  
  mask = zeros(KL.ctr,1);
  
  for iter=1:KL.ctr
    if (KL.idx(iter,1) == 0 || KL.idx(iter,2) == 0)
      continue %no neighbors
    end
    
    kn = KL.idx(iter,1);
    pn = KL.idx(iter,2);
    
    if (KL.rho(kn,1) - KL.rho(kp,1)) > dot(KL.rho(kn,2), KL.rho(kp,2))
      continue %uncertainty test
    end
    
    alpha = ( KL.grad(kn,1)*KL.grad(kp,1) + KL.grad(kn,2)*KL.grad(kp,2) ) / ...
    (dot(KL.grad(kn,:)) * dot(KL.grad(kp,:)));
    
    if alpha - thresh < 0
      continue
    end
    
    alpha = (alpha-thresh)/(1-thresh);
    wr = 1/(KL.rho(iter,2)^2);
    wrn = alpha/(KL.rho(kn,2)^2);
    wrp = alpha/(KL.rho(kp,2)^2);
    
    r(iter) = ( KL.rho(iter,1) * wr + ...
                KL.rho(kn,1) * wrn + ...
                KL.rho(kp,1) * wrp ) / ...
                (wr + wrn + wrp);
    s(iter) = ( KL.rho(iter,2) * wr + ...
                KL.rho(kn,2) * wrn + ...
                KL.rho(kp,2) * wrp ) / ...
                (wr + wrn + wrp);  
      
    mask(iter) = 1;
    r_num++;    
  end
  for (iter=1:KL.ctr)
    if(mask(iter))
      KL.rho(iter,1) = r(iter);
      KL.rho(iter,2) = s(iter);
    end
  end
  
  return r_num;
end

function [KL] = UpdateInverseDepthKalman(...
 Vel, RVel, RW0, KL) % 1e-5
 
  for (iter = 1:KL.ctr)
    if KL.matching(iter) >=0
    
    zf = conf.zf;
    
    %debug cout, pomijam
    KL.rhoPredict(iter,2) = KL.rho(iter,2);
    
    qx = KL.posImage(iter,2);
    qy = KL.posImage(iter,1);
    
    q0x = KL.posImageMatch(iter,2);
    q0y = KL.posImageMatch(iter,1);
    
    v_rho = KL.rho(iter,2)^2;
    
    u_x = KL.matchedGrad(iter,2) / ...
          dot(KL.matchedGrad(iter,:), KL.matchedGrad(iter,:));
    u_y = KL.matchedGrad(iter,1) / ...
          dot(KL.matchedGrad(iter,:), KL.matchedGrad(iter,:));
    
    Y = u_x * (qx-q0x) + u_y*(qy-q0y);
    H = u_x * (Vel(2)*zf - Vel(3)*q0x) + ...
        u_y * (Vel(1)*zf - Vel(3)*q0y);
        
    rho_p = 1 / ( 1/KL.rh(iter,1) + Vel(3) ); %inv depth
    KL.rhoPredict(iter,1) = rho_p;
    
    F = 1 / (1+ KL.rho(iter,1) * Vel(3)); %jacobian
    F *= F;
    p_p = F*v_rho*F + (KL.rho(iter,1)*conf.RESHAPE_Q_RELATIVE)^2 + ...
          rho_p^2*RVel(3,3)*rho_p^2 + conf.RESHAPE_Q_ABSOLUTE^2;
          
    e = Y-H*rho_p
    Mk = [ -1 ,
          u_x*rho_p*zf ,
          u_y*rho_p*zf ,
          -rho_p*(u_x*q0x + u_y*q0y) ,
          u_x*(rho_p*Vel(3) ,
          u_y*rho_p*Vel(3) ];
    
    R = zeros(6,6);
    loc_unc_sq = conf.LOCATION_UNCERTAINTY^2;
    R(1,1) = loc_unc_sq;
    R(2:4,2:4) = RVel;
    R(5,5) = loc_unc_sq;
    R(6,6) = loc_unc_sq;
    
    S = H*p_p*H + (Mk*R*Mk');
    K = p_p*H*(1/S);
    KL.rho(iter,1) = rho_p + K*e;
    v_rho = (1-K*H) * p_p;
    KL.rho(iter,2) = sqrt(v_rho);
    
    if(KL.rho(iter,1) < conf.S_RHO_MIN)
      KL.rho(iter,2) += conf.S_RHO_MIN - KL.rho(iter,1);
      KL.rho(iter,1) = conf.S_RHO_MIN;
    else if KL.rho(iter,1) > conf.S_RHO_MAX
      KL.rho(iter,1) = conf.S_RHO_MAX;
    else if sum(isnan(KL.rho(iter,:))) | ...
            sum(isinf(KL.rho(iter,:)))
      KL.rho(iter,1) = conf.RHO_INIT;
      KL.rho(iter,2) = conf.S_RHO_MAX;
    else if KL.rho(iter,2) < 0
      KL.rho(iter,1) = conf.RHO_INIT;
      KL.rho(iter,2) = conf.S_RHO_MAX;
    end
  
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
      KL.rho(iter,2) <= 0 || ...
      KL.rho(iter,2) > s_rho_min
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

  P_Kp = 1./rTr;

  % re_escale is always true
  KL.rho /= Kp;
  
end

