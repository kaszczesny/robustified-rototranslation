1;

function [] = VisualizeMatches( KL_prev, KL, use_m_id )
  global conf;
  
  if use_m_id
    title_string = "matches m id";
  else
    title_string = "matches m id f";
  end    
    
    %figure(10 + 2*use_m_id)
    pos1 =[];
    pos2 =[];
    imsize = conf.imgsize(end:-1:1); %xyz to yxz
    
    im1 = imread(conf.im_name(KL_prev.frame_id));
    if length(size(im1)) == 3
      im1 = rgb2gray(im1);
    end
    im1 = imresize(im1, conf.scale);
    %{
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
    %}
    
    %keyboard("<<<")
    
    %close
    
    im1_ = cat(3, im1, im1, im1);
    for iter = 1:KL_prev.ctr
      im1_( KL_prev.pos(iter, 2), KL_prev.pos(iter,1), : ) = [255 0 0];
    end
    for iter = 1:KL.ctr
      if im1_( KL.pos(iter, 2), KL.pos(iter,1), 1 ) == 255
        red = 255;
      else
        red = 0;
      end  
      im1_( KL.pos(iter, 2), KL.pos(iter,1), : ) = [red 0 255];
    end
    
    save_img(im1_, KL.frame_id, 11 + 2*use_m_id);
    
    if conf.visualize_matches %&& use_m_id == 1
      figure(11 + 2*use_m_id)
      
      imshow(im1_);
      hold on
      
      [y x] = meshgrid(1:conf.imgsize(1), 1:conf.imgsize(2));
      
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
      
      quiver(y, x, vec_y, vec_x, 0, 'color', [0.3 1 0.3]);
      
      hold off;
      title(title_string);
    end
    
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
end

function [] = VisualizeDepth(KL_prev)
  global conf;
    
    im_plot = zeros(conf.imgsize);
    %q = quantile(1./KL_prev.rho(:,1), 0.975);
    q = 1./conf.S_RHO_MIN;
    
    for iter = 1:KL_prev.ctr
      if KL_prev.matching(iter) < 0
        continue
      end  
      if 1./KL_prev.rho(iter,1) < q
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 1./KL_prev.rho(iter,1);
      else
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = q;
      end
    end
    
    save_img(besos(im_plot, 0, q), KL_prev.frame_id, 14);
    
    if conf.visualize_depth
      figure(14);
      imagesc(im_plot');
      %axis equal; colormap cubehelix; colorbar;
      axis equal; colormap jet; colorbar;
      title('depth')
      pause(0)
    end
end

function [] = VisualizeDepthVar(KL_prev)
  global conf;
    
    im_plot = zeros(conf.imgsize);
    q = 1./conf.S_RHO_MIN;
    
    for iter = 1:KL_prev.ctr
      if KL_prev.matching(iter) < 0
        continue
      end  
      if 1./KL_prev.rho(iter,2) < q
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = 1./KL_prev.rho(iter,2);
      else
        im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = q;
      end
    end
    
    save_img(besos(im_plot, 0, q), KL_prev.frame_id, 25);
    
    if conf.visualize_depth
      figure(25);
      imagesc(im_plot');
      %axis equal; colormap cubehelix; colorbar;
      axis equal; colormap jet; colorbar;
      title('depth uncertainty')
      pause(0)
    end  
end

function [] = VisualizeHistory(KL_prev)
  global conf;
  
    im_plot = zeros(conf.imgsize);
    
    for iter = 1:KL_prev.ctr
      if KL_prev.matching(iter) < 0
        continue
      end  
      im_plot(KL_prev.pos(iter,1), KL_prev.pos(iter,2)) = KL_prev.frames(iter);
    end
    
    save_img(besos(im_plot, 0, 25), KL_prev.frame_id, 15);
    
    if conf.visualize_history
      figure(15)
      imagesc(im_plot')
      axis equal; colormap jet; colorbar;
      title('history')
    end
end

function [KL_prev] = VisualizeDepth3D(KL_prev)
  global conf;
    
    % todo untangle visualization from filter
    % todo median over larger neighborhood
    
    edge_id = zeros(1,KL_prev.ctr);
    figure(19)
    clf
    hold on;
    
    rho_copy = KL_prev.rho(:,1);
    
    for iter = 1:KL_prev.ctr
      if edge_id(iter) == 1
        continue;
      end
      
      %making a vector of ids
      id = iter;
      edge_id(id) = 1;
      nid = KL_prev.idx(iter,2);
      pid = KL_prev.idx(iter,1);
      edgevec = id;
      
      %nid
      while nid ~= 0
        stepid = id;
        id = nid;
        %check for prior iterations
        if edge_id(id) == 1
          break
        end
        %check for loop
        if KL_prev.idx(id,1) == KL_prev.idx(id,2)
          edge_id( KL_prev.idx(id,1) ) = 1;
          edge_id( KL_prev.idx(id,2) ) = 1;
          break
        end
        %check for change in idxses
        if KL_prev.idx(id, 2) == stepid;
          nid = KL_prev.idx(id, 1);
        else nid = KL_prev.idx(id, 2);
        end
        
        edgevec = [edgevec, id];
        edge_id(id) = 1;
      end
      
      %pid
      while pid ~= 0
        stepid = id;
        id = pid;
        %check for prior iterations
        if edge_id(id) == 1
          break
        end
        %check for loop
        if KL_prev.idx(id,1) == KL_prev.idx(id,2)
          edge_id( KL_prev.idx(id,1) ) = 1;
          edge_id( KL_prev.idx(id,2) ) = 1;
          break
        end
        %check for change in idxses
        if KL_prev.idx(id, 1) == stepid;
          pid = KL_prev.idx(id, 2);
        else pid = KL_prev.idx(id, 1);
        end
        
        edgevec = [id, edgevec];
        edge_id(id) = 1;
      end
     
      %make a coordinate vector
      coor = zeros(3,0);
      for iter = 1:length(edgevec)
        if KL_prev.matching(edgevec(iter)) < 0
          continue;
        end  
        if 1/KL_prev.rho(     edgevec(iter),1 ) > 20
          depth = 20;
        else
          depth = 1/KL_prev.rho(     edgevec(iter),1 );
        end
        coor = [coor, [ KL_prev.posImage( edgevec(iter),1 ); ...
                        depth                               ; ...
                        KL_prev.posImage( edgevec(iter),2 )] ];
      end
      %median filtering
      medtemp = coor(2,:);
      if size(coor, 2) >= 5
        for iter = 3:size(coor,2)-2
          depthvec = coor(2,iter-2:iter+2);
          depthvecs = sort(depthvec);
          med = median(depthvecs);
        
          %rewrite to KL_prev
          rho_copy(edgevec(iter),1) = 1/med;
          medtemp(iter) = med;
        end
      end
      coor(2,:) = medtemp;
      
      
        %visualize that edge
        view(3)
        plot3(coor(1,:)./conf.zf.*coor(2,:), coor(2,:), coor(3,:)./conf.zf.*coor(2,:), 'x.-', ...
              0, 0, 0, 'r.');     
      
    end
    hold off;
    set(gca,'zdir', 'reverse')
    view(0,90)
    
    KL_prev.rho(:,1) = rho_copy;
    
end