1;

function [] = VisualizeMatches( KL_prev, KL, use_m_id )
  global conf;
  
  if use_m_id
    title_string = "matches m id";
  else
    title_string = "matches m id f";
  end    
    
    figure(10 + 2*use_m_id)
    pos1 =[];
    pos2 =[];
    imsize = conf.imgsize(end:-1:1); %xyz to yxz
    
    im1 = imread(conf.im_name(KL_prev.frame_id));
    if length(size(im1)) == 3
      im1 = rgb2gray(im1);
    end
    im1 = imresize(im1, conf.scale);
    
    im2 = imread(conf.im_name(KL.frame_id));
    if length(size(im2)) == 3
      im2 = rgb2gray(im2);
    end
    im2 = imresize(im2, conf.scale);
    
    if use_m_id
      for iter = 1:conf.visualize_matches_step:KL.ctr
        if KL.matching(iter) != -1
          pos1(end+1,:) = KL_prev.pos(KL.matching(iter),:);
          pos2(end+1,:) = KL.pos(iter,:);
        end
      end
    else
      for iter = 1:conf.visualize_matches_step:KL_prev.ctr
        if KL_prev.forward(iter) != -1
          pos1(end+1,:) = KL_prev.pos(iter,:);
          pos2(end+1,:) = KL.pos(KL_prev.forward(iter),:);
        end
      end
    end  

    im_plot = [im1, im2; im2, zeros(imsize)];
    imshow(im_plot);
    hold on;

    for iter = 1:size(pos1,1)
      color = rand(1,3);
      plot([pos1(iter,1), pos2(iter,1)+imsize(2)], ...
            [pos1(iter,2), pos2(iter,2)], "-o", ...
            'Color', color, 'markerfacecolor', color);
      plot([pos1(iter,1), pos2(iter,1)], ...
            [pos1(iter,2), pos2(iter,2)+imsize(1)], "-o", ...
            'Color', color, 'markerfacecolor', color);
    end
    hold off;
    title(title_string)
    
    
    %keyboard("<<<")
    
    %close
    figure(11 + 2*use_m_id)
    
    
    [y x] = meshgrid(1:conf.imgsize(1), 1:conf.imgsize(2));
    im1_ = im1;
    for iter = 1:KL_prev.ctr
      im1_( KL_prev.pos(iter, 2), KL_prev.pos(iter,1) ) = 255;
    end
    for iter = 1:KL.ctr
      im1_( KL.pos(iter, 2), KL.pos(iter,1) ) = 0;
    end
    imshow(im1_);
    hold on
    vec_y = zeros(conf.imgsize(2:-1:1));
    vec_x = zeros(conf.imgsize(2:-1:1));
    for iter = 1:KL.ctr
      if KL.matching(iter) != -1
        match = KL.matching(iter);
        vec =  KL.pos(iter,:) - KL_prev.pos(match,:);
        vec_y( KL_prev.pos(match,2), KL_prev.pos(match,1) ) = vec(1);
        vec_x( KL_prev.pos(match,2), KL_prev.pos(match,1) ) = vec(2);
      end
    end
    quiver(y, x, vec_y, vec_x, 0);
    
    %{
    imshow(im1);
    hold on;

    for iter = 1:size(pos1,1)
      color = rand(1,3);
      plot([pos1(iter,1), pos2(iter,1)], ...
            [pos1(iter,2), pos2(iter,2)], "-", ...
            'Color', color, 'markerfacecolor', color);
      plot(pos1(iter,1), pos1(iter,2), "o", ...
            'Color', color, 'markerfacecolor', color);
    end
    %}
    hold off;
    title(title_string);
end

function [] = VisualizeDepth(KL_prev)
  global conf;
    
    im_plot = zeros(conf.imgsize) - 5;
    %q = quantile(1./KL_prev.rho(:,1), 0.975);
    q = 20;
    
    for iter = 1:KL_prev.ctr
      if 1./KL_prev.rho(iter,1) < q
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 1./KL_prev.rho(iter,1);
      else
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = q*1.5;
      end
    end
    figure(14);
    imagesc(im_plot');
    %axis equal; colormap cubehelix; colorbar;
    axis equal; colormap jet; colorbar;
    title('depth')
end

function [] = VisualizeHistory(KL_prev)
  global conf;
  
    im_plot = zeros(conf.imgsize);
    
    for iter = 1:KL_prev.ctr
      im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = KL_prev.frames(iter);
    end  
    figure(15)
    imagesc(im_plot')
    axis equal; colormap jet; colorbar;
    title('history')
end