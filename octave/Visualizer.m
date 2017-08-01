1;

function [] = VisualizeMatches( KL_prev, KL )
  conf = Config();
    
    figure()
    pos1 =[];
    pos2 =[];
    imsize = conf.imgsize;
    
    im1 = zeros(imsize);
    im2 = zeros(imsize);
    for iter = 1:KL_prev.ctr
      im1(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 1;
    end
    for iter = 1:KL.ctr
      im2(KL.pos(iter,1), KL.pos(iter,2)) = 1;
    end
    
    for iter = 1:KL.ctr
    if KL.matching(iter) != -1
      pos1(end+1,:) = KL_prev.pos(KL.matching(iter),:);
      pos2(end+1,:) = KL.pos(iter,:);
      end
    end

    im_plot = [im1, im2; im2, zeros(imsize)];
    imshow(im_plot);
    hold on;

    for iter = 1:size(pos1,1)
      color = rand(1,3);
      plot([pos1(iter,2), pos2(iter,2)+imsize(2)], ...
            [pos1(iter,1), pos2(iter,1)], "-o", ...
            'Color', color, 'markerfacecolor', color);
      plot([pos1(iter,2), pos2(iter,2)], ...
            [pos1(iter,1), pos2(iter,1)+imsize(1)], "-o", ...
            'Color', color, 'markerfacecolor', color);
    end
    hold off;
    
    
    figure()
    imshow(im1);
    hold on;

    for iter = 1:size(pos1,1)
      color = rand(1,3);
      plot([pos1(iter,2), pos2(iter,2)], ...
            [pos1(iter,1), pos2(iter,1)], "-", ...
            'Color', color, 'markerfacecolor', color);
      plot(pos1(iter,2), pos1(iter,1), "o", ...
            'Color', color, 'markerfacecolor', color);
    end
    hold off;
end

function [] = VisualizeDepth(KL_prev)
  conf = Config();
    
    im_plot = zeros(conf.imgsize);
    q = quantile(1./KL_prev.rho(:,1), 0.975);
    
    for iter = 1:KL_prev.ctr
      if 1./KL_prev.rho(iter,1) < q
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 1./KL_prev.rho(iter,1);
      else
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 0;
      end
    end
    figure();
    imagesc(im_plot);
    axis equal; colormap jet; colorbar;
end

function [] = VisualizeHistory(KL_prev)
  conf = Config();
  
    im_plot = zeros(conf.imgsize);
    
    for iter = 1:KL_prev.ctr
      im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = KL_prev.frames(iter);
    end  
    figure
    imagesc(im_plot)
    axis equal; colormap jet; colorbar;
  
end