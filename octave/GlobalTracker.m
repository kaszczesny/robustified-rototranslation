function [ ...
  ret,
  Vel, W0, RVel, RW0, ...
  rel_error, rel_error_score, MatchNumThresh
] = GlobalTracker (...
    Vel, ... %initial translation estimation (3 vector; init with zeros)
    W0, ... %initial rotation estimation (3 vector; init with zeros)
    RVel, %uncertainty Model if the initial Vel estimate will be used as prior (3x3 matrix; init with eye*1e50)
    RW0,  %uncertainty Model if the initial W0  estimate will be used as prior (3x3 matrix; init with eye*1e-10)
    KLidx, KLrho, ...
    rel_error, ... % Estimated relative error on the state (init with zero)
    rel_error_score, ... % Estimated relative error on the score (init with 0)
)
  % based on Minimizer_RV
  % performs Equation (8) minimization
  % returns total energy (?)
  % todo: Check for minimization errors
  
function [s_rho] = EstimateQuantile(KLrho)
  % based on edge_tracker EstimateQuantile
  % tells which s_rho (uncertainty) is such that S_RHO_MIN-s_rho == quantile
  
  % todo: Config.m
  S_RHO_MIN = 1e-3; % starting uncertainty of histogram in EstimateQuantile
  S_RHO_MAX = 20; % final uncertainty of histogram in EstimateQuantile
  PERCENTILE = 0.9; % quantile threshold in EstimateQuantile
  N_BINS = 100; %number of histogram bins in EstimateQuantile
  
  
  bins = zeros(N_BINS, 1);

  for iter = 1:size(KLrho, 1)
    %check in which bin s_rho is and increment its counter
    idx = floor( N_BINS * (KLrho(iter,2)-S_RHO_MIN) / (S_RHO_MAX - S_RHO_MIN) ) + 1;
    if idx < 1
      idx = 1;
    elseif idx > N_BINS
      idx = N_BINS;
    end  
    bins(idx) += 1;
  end

  s_rho = 1e3; % todo why 1e3; move to Config.m
  a = 0; %accumulator

  for iter = 1:N_BINS
    if (a > PERCENTILE * size(KLrho, 1) )
      s_rho = (iter - 1) * (S_RHO_MAX-S_RHO_MIN) / N_BINS + S_RHO_MIN;
      return
    end
    a += bins(iter);
  end
end

function [...
    score, ...
    JtJ, JtF, KLm_id_forward, ...
    P0m, DResidualNew
] = TryVelRot(
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
    KLpos, KLrho, ... % keylines
    P0m, ... % linear transpose coodinates vector of 3D klist positions (vector or pnum*3)
    ... % pnum, ... % number of points
    max_s_rho, ... % estimated rho 0.9 quantile
    DResidual, ... % Last iteration Distance Residuals - for calculating weights, vector pnumx1
    DResidualNew, ... % New iteration Distance Residuals - new residuals, vector pnumx1
    
)
  % returns energy based on dot product of distance residuals
  pnum = size(KLpos,1);
  score = 0;
  JtJ = zeros(6,6);
  JtF = zeros(6,1);
  
  %{
    linear transpose coodinates vectors (LTCV) are arranged as follows:
    {x1 x2 x3 .... xn y1 y2 y3 ... yn z1 z2 z3 ... zn}
    where x & y are KL subpixel position in homogeneous coordinates (plane on focal length zf)
    and z is rho (estimated inverse depth)
    Then they are projected into 3D by Equation 4
  %}
  
  MATCH_THRESH = 1.; % Match Gradient Treshold
  MATCH_NUM_THRESH = 2; % minimal number of frames KL has to be on to be considered stable (exluding very first MATCH_NUM_THRESH frames)
  REWEIGHT_DISTANCE = 2.; % Distance cut-off for reweigthing (k_hubber/k_huber)
  
  % todo: rozciagnij vel i rot do macierzy
  
  Ptm = zeros(pnum, 3); %LTCV of transformed 3d coordinates
  PtIm = zeros(pnum, 3); %LTCV of transformed image coordinates
  fi=0;
  
  for iter = 1:pnum
    % todo: resetuj forward matching
    
    fm = zeros(pnum, 1); %weighted residuals
    df_dPi = zeros(pnum, 2); %weighted img derivative of residual
    if (KLrho(iter,2) > max_s_rho % lub jesli ten keyline nie jest na wystarczajacej ilosci klatek
      
      fm(iter) = 0;
      df_dPi(iter, :) = 0;
      continue;
      
    end
    
    % todo: convert to img coordinates
    %       te punkty zaokragl do integerow
    x = KLpos(iter, 2);
    y = KLpos(iter, 1);
    
    weight = 1;
    if( ReWeight & abs(DResidual(iter) > REWEIGHT_DISTANCE )
      weight = REWEIGHT_DISTANCE ./ abs(DResidual(iter);
    end
    
    if ( x<2 | y<2 | x>=imgsize(2) | y>=imgsize(1) )
      fm(iter) = max_r/KLrho(iter, 2);
      
      if(ReWeight)
        fm(iter) *= weight;
      end
      df_dPi(iter, :) = 0;
      DResidualNew(iter) = max_r;
      continue;
    end
    
    % todo : jakies cos z obracaniem, nie czaje tego, wyjasnienie
    
    % todo: funkcja Calc_f_J :D //residuals and gradients
    %fm(iter) = Calc_f_J_XD
    
    if(ReWeight)
      fm(iter) *= weight;
      df_dPi(iter, :) *= weight;
    end
    
    DResidualNew(iter) = fi %zwracane z Calc_f_J, chyba, bo inaczej 0
    
  end
  
  if(ProcJF)
    
    Jm = zeros(pnum, 6);
    RhoTmp0 = zeros(pnum, 1);
    
    RhoTmp0 = zf * PtIm(:, 3); %z coordinate constant mul
    
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
 
  if(UsePriors)
    % todo: uzyj sobie, zawsze false
  end
                                
end                                 
  
  % todo: gracefully move constants to Config.m
  % UsePriors is always false
  ITER_MAX = 10; % iterations of Farquad probably Eq. (9)
  % init_type is always 2
  max_s_rho = EstimateQuantile(KLrho);
  INIT_ITER = 2; % Actually controls ProcJF in TryVelRot (true or false depending on interation)
  

  if size(KLidx, 1) < 1
    ret = 0;
    return
  end

  

end
