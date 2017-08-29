1;

function [score, R, T] = fit3d(do_plot, scale, points_groundtruth, points_algo, ret_mean)
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

    if do_plot
      figure(18)
      plot3( trans(1,:), trans(3,:), trans(2,:), 'k.-',
        p(1,:), p(3,:), p(2,:), 'b.-', ...
        p_(1,:), p_(3,:), p_(2,:), 'r.-');
      axis equal
      
      %
      figure(50)
      subplot(311)
      plot(p_(1,:));
      hold on;
      plot(trans(1,:)','r');

      subplot(312)
      plot(p_(2,:));
      hold on;
      plot(trans(2,:)','r');

      subplot(313)
      plot(p_(3,:));
      hold on;
      plot(trans(3,:)','r');
      %
    end

    if ~ret_mean
      score = sum(sum((trans-p_).^2,1));
    else
      score = mean(sum((trans-p_).^2,1));
    end
end

function [scale, score, R, vel] = FitTrajectory_( gt, Pos )


  f = @(s) fit3d(0, s, gt,Pos, 0);


  [scale, score] = fminsearch(f,1);
  if isinf(score)
    scale = 1;
  end
  score_sum = score
  [score, R, vel] = fit3d( 1, scale, gt, Pos, 1 );

  l = logm(R);
  rot = [l(2,3) -l(1,3) l(1,2)];
  vel = vel';
end