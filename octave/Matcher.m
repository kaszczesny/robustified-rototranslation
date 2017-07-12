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