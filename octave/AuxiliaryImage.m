function [distance_field, KLidx_field] = AuxiliaryImage (KL)

conf = Config();

distance_field = zeros(conf.imgsize);
KLidx_field = zeros(conf.imgsize) - 1;

for idx = 1:KL.ctr
  for t=-conf.MAX_R:conf.MAX_R
    x = KL.grad(idx, 2) * t + KL.posSubpix(idx, 2);
    y = KL.grad(idx, 1) * t + KL.posSubpix(idx, 1);
    
    x = round(x);
    y = round(y);
    
    if x < 1 || y < 1 || x > conf.imgsize(2) || y > conf.imgsize(1)
      continue; % out of border
    end
  
    at = abs(t);  
    if KLidx_field(y,x) >= 0 && at > distance_field(y,x)
      continue;
    end
  
    distance_field(y,x) = at;
    KLidx_field(y,x) = idx;  
    
  end
end

if conf.visualize
  figure; imagesc(distance_field);axis equal
end
