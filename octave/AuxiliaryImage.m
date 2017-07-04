function [distance_field, KLidx_field] = AuxiliaryImage (imsize, KLctr, KLpos, KLposSubpix, KLidx, KLgrad)

conf = Config();

distance_field = zeros(imsize);
KLidx_field = zeros(imsize) - 1;

for idx = 1:KLctr
  for t=-conf.MAX_R:conf.MAX_R
    x = KLgrad(idx, 2) * t + KLposSubpix(idx, 2);
    y = KLgrad(idx, 1) * t + KLposSubpix(idx, 1);
    
    x = round(x);
    y = round(y);
    
    if x < 1 || y < 1 || x > imsize(2) || y > imsize(1)
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
