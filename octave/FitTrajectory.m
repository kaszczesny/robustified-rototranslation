1;

function [score, R, T] = fit3d(scale, points_groundtruth, points_algo)
    %global conf;

    p_ = points_groundtruth;
    p = points_algo * scale;
    % centroids
    p_c = mean(p,2);
    p__c = mean(p_,2);

    q = p - p_c;
    q_ = p_ - p__c;

    H = q * q_';

    [u s v] = svd(H);

    X = v*u';

    if abs(det(X) + 1) < 1e-6
      score = Inf;
    end

    R = X;
    T = p__c - R*p_c;

    trans = R*p + T;

    %if conf.visualize_RT
      figure(18)
      plot3( trans(1,:), trans(3,:), trans(2,:), 'k.-',
        p_(1,:), p_(3,:), p_(2,:), 'r.-');
      axis equal
    %end

    score = sum(sum((trans-p_).^2,1));
end

function [scale, score, vel, rot] = FitTrajectory_( gt, Pos )


  f = @(s) fit3d(s, gt,Pos);


  [scale, score] = fminsearch(f,1);
  if isinf(score)
    scale = 1;
  end   
  [~, R, vel] = fit3d( scale, gt, Pos );

  l = logm(R);
  rot = [-l(1,2) l(1,3) -l(2,3)];
  vel = vel';
end