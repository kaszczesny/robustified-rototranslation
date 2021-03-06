function [distance_field, KLidx_field] = AuxiliaryImage (KL)

global conf;

distance_field = zeros(conf.imgsize) - 1; % init with -1 for better visualization
KLidx_field = zeros(conf.imgsize) - 1;

for idx = 1:KL.ctr
  for t=-conf.MAX_R:0.5:conf.MAX_R % floating increment will be slower, but looks so much better
    % todo: check if floating increment gives better results
    
    u_m = KL.vers(idx,:);
    x = u_m(1) * t + KL.posSubpix(idx, 1);
    y = u_m(2) * t + KL.posSubpix(idx, 2);
    
    x = round(x);
    y = round(y);
    
    if x < 1 || y < 1 || x > conf.imgsize(1) || y > conf.imgsize(2)
      continue; % out of border
    end
  
    at = abs(t);  
    if KLidx_field(x,y) >= 0 && at > distance_field(x,y)
      continue;
    end
  
    distance_field(x,y) = at;
    KLidx_field(x,y) = idx;  
    
  end
end


  viz = -distance_field;
  viz(viz == 1) = -conf.MAX_R - 2;
  save_img(besos(viz, -conf.MAX_R - 2, 0), KL.frame_id, 4);
if conf.visualize_edges
  figure(4);
  imagesc(viz');axis equal;colorbar
  title('auxiliary')
end
