function [KL, img_mask] = EdgeFinder(frame_id, use_depth)
%internally uses yxz notation
%returns xyz to struc


%% CONFIG %%
global conf;
win_s = conf.win_s;


persistent thresh_grad = conf.THRESH_GRAD_MIN;
persistent l_kl_num = 0;


im_name = conf.im_name(frame_id);
im_name_depth = conf.im_name_depth(frame_id);

% UpdateThresh %
thresh_grad -= conf.THRESH_GRAD_GAIN * (conf.KL_REFERENCE_NUMBER - l_kl_num);
thresh_grad = max(conf.THRESH_GRAD_MIN, thresh_grad);
thresh_grad = min(conf.THRESH_GRAD_MAX, thresh_grad);

%% DOG %%
im = imread(im_name);
if length(size(im)) == 3
  im = rgb2gray(im);
end

im = imresize(im, conf.scale);
imc = im(2:end-1,2:end-1); %rectification correction

im_blurred1 = double(cv.GaussianBlur(
              imc, "KSize", [conf.ksize,conf.ksize], 
              "SigmaX", conf.sigma1, "SigmaY", conf.sigma1));
im_blurred2 = double(cv.GaussianBlur(
              imc, "KSize", [conf.ksize,conf.ksize],
              "SigmaX", conf.sigma2, "SigmaY", conf.sigma2));
              %% ratio of kernels should be 4:1 or 5:1
              

doggy = double(im_blurred2) - double(im_blurred1);
dog = zeros(conf.imgsize(2:-1:1));
dog(2:end-1,2:end-1) = doggy;

if use_depth
  depth = imresize(imread(im_name_depth), conf.scale);
  depth_mask = depth > 0; % depth == 0 means 100% uncertainty
  %depth = double(cv.GaussianBlur(depth, "KSize", [conf.ksize,conf.ksize]));
  depth = double(depth);
  depth /= 5000; % TUM scaling factor
  depth_mask(depth > conf.cheat_upper_bound) = 0;
  depth_mask(depth < conf.cheat_lower_bound) = 0;
  depth = 1./ depth; %inverse depth
end


%% KEYLINE INIT %%
%{
m_m     KLgrad (2d f)
u_m     normalized KLgrad (2d f)
        KLvers (like "versor")
n_m     norm of KLgrad (f)
        KLnorm

c_p     KLposSubpix (2d f)
round(c_p)  KLpos

rho     Estimated Inverse Depth (f)
s_rho   Estimated Inverse Depth Uncertainty (f)
        KLrho

rho0    Predicted Inverse Depth in EKF (use only if rescaling) (f)
s_rho0  Predicted Inverse Depth Uncertainty (f)
        KLrhoPredict

p_m     Subpixel position in homo coordinates (plane on focal length zf) (2d f)
        our homo coordinates are also normalized by zf
        KLposImage
p_m_0   matched KL (subpixel) position in homo coordinates (2d f)
        KLposImageMatch

m_id    Id of the matching KL
        KLmatching
m_id_f  Id of the matching KL by forward matching
        KLforward

m_num   Number of consecutive matches (int)
        KLframes

m_m0    Gradient of matched KL (2d f)
        KLmatchedGrad
n_m0    KLmatchedNorm (f)

p_id    ID of previous KL (int)
n_id    ID of next KL (int)
        KLidx
        
net_id  Network ID of Keyline
        ####### 

%}
KLctr = 0;
KLpos = []; % pixel position
KLposSubpix = []; % subpixel position
KLposImage = []; % subpixel position in image coordinates
KLposImageMatch = []; % matched KL (subpixel) position in homo coordinates
KLidx = []; % index of previous/next keyline
KLgrad = []; % local third derivative vector
KLvers = []; % local third derivative versor
KLnorm = []; % local third derivative vector norm
KLrho = []; % estimated inverse depth and inverse depth uncertainty
KLrhoPredict = []; % predicted inverse depth and inverse depth uncertainty
KLmatching = []; % id of the matching keyline
KLforward = []; % id of forward match
KLframes = []; % number of consecutive matches
KLmatchedGrad = []; % gradient of matched gradient
KLmatchedNorm = []; % norm of matched gradient

img_mask = zeros(size(im));


%% TESTS INIT %%
edge_probability = dog*0; %not normalized to [0,1]
xs_im = dog*0;
ys_im = dog*0;
vec_x = dog*0;
vec_y = dog*0;

  %gradient for test 1
  % todo switch x with y?
img_dx = cv.filter2D(im_blurred2, [0 -1 0; 0 0 0; 0 1 0]');
img_dy = cv.filter2D(im_blurred2, [0 -1 0; 0 0 0; 0 1 0] );
% there was concern whether this should be sqrt'ed or not, now seems better (at least for current dataset)
n2gI = img_dx.^2 + img_dy.^2;

  %for test 2
%Y = zeros((win_s*2+1).^2, 1);
Phi = zeros( (win_s*2+1).^2, 3 );
k = 1;
for yter = -win_s:win_s
  for xter = -win_s:win_s
    Phi( k, : ) = [xter yter 1];
    k++;
  end
end
PInv = pinv( Phi );

%% EDGE FINDER LOOP %%
for yter = 1+win_s:size(dog, 1)-win_s
  for xter = 1+win_s:size(dog, 2)-win_s
    % Test 1: local gradient must be sufficiently stronk
    if n2gI(yter,xter) < (thresh_grad*conf.max_img_value).^2;
      edge_probability(yter,xter) = 1;
      continue
    end
    
    % Test 2: current point is (more or less) in the middle of a keyline
    Y = dog(yter+[-win_s:win_s], xter+[-win_s:win_s]);
    pn = sum(sign(Y));
    if abs(pn) > conf.PosNegThresh * (win_s*2+1).^2
      edge_probability(yter,xter) = 2;
      continue
    end  
    
    % Test 2.(9): bad conditioning
    theta = PInv * Y'(:); % solution of equation system
    if sum([theta([1, 2]).^2]) < 1e-6
      edge_probability(yter,xter) = 3;
      continue
    end
    
    % Test 3: subpixel position
    xs =-theta(1)*theta(3) / (theta(1)^2 + theta(2)^2); %subpixel position
    ys =-theta(2)*theta(3) / (theta(1)^2 + theta(2)^2); %subpixel position
    
    % todo: limit horrendeous values (if abs(a) > sth then a = sth*sign(a))
    
    xs_im(yter,xter) = xs;
    ys_im(yter,xter) = ys;
    
    
    if conf.visualize_crossing
      disp([yter xter])
      VisualizeZeroCrossing(Y, theta, im_blurred2(yter+[-win_s:win_s], xter+[-win_s:win_s]))
      input('...');
      close;
    end
    
    % zero crossing inside or outside pixel area
    if max(abs([ys xs])) > 0.5;
      edge_probability(yter,xter) = 4;
      continue
    end
    
    % a test on dog was missing
    n2_m = norm(theta(1:2)).^2;
    if n2_m < conf.detector_dog_thresh * thresh_grad * conf.max_img_value
      edge_probability(yter,xter) = 5;
      continue
    end
    
    if use_depth
      if depth_mask(yter,xter)
        rho = depth(yter,xter);
      else
        edge_probability(yter,xter) = 6;
        continue
      end
    else
      rho = conf.RHO_INIT;  
    end  
    
    % local normal vector, normalized to versor after projection to z=0
    n = [-theta(1) -theta(2) 1];
    %n = n ./ sqrt(sum(n([1 2]).^2));
    vec_x(yter,xter) = n(2); %weirdly, n(1) and n(2) are swapped
    vec_y(yter,xter) = n(1);
    
    edge_probability(yter,xter) = 7;
  
    KLctr += 1;
    KLpos = [KLpos; yter, xter];
    KLposSubpix = [KLposSubpix; ys+yter, xs+xter];
    %KLposImage = [KLposImage; [ys+yter, xs+xter]-conf.principal_point(2:-1:1)];
    %KLidx = [KLidx; 0, 0];
    KLgrad = [KLgrad; theta([2 1])'];
    KLnorm = [KLnorm; sqrt(n2_m)];
    KLvers = [KLvers; KLgrad(end,:) ./ n2_m];
    KLrho = [KLrho; rho, conf.S_RHO_MAX];
    %KLrhoPredict = [KLrhoPredict; 0, 0];
    %KLmatching = [KLmatching; -1];
    %KLforward = [KLforward; -1];
    %KLframes = [KLframes; 0];
    %KLmatchedGrad = [KLmatchedGrad; 0, 0];
    %KLmatchedNorm = [KLmatchedNorm; 0];
    img_mask( yter, xter ) = KLctr;
  end
end

KLidx = zeros(KLctr,2);
KLrhoPredict = zeros(KLctr,2);
KLmatching = zeros(KLctr,1) -1;
KLforward = zeros(KLctr,1) -1;
KLframes = zeros(KLctr,1);
KLmatchedGrad = zeros(KLctr,2);
KLmatchedNorm = zeros(KLctr,1);

KLposImage = pixelToNormalized(KLposSubpix(:,[2 1]));
KLposImage = KLposImage(:, [2 1]);
KLposImageMatch = KLposImage;


%% KEYLINE JOINING %%
[KLidx, KLref] = KeylineJoiner((edge_probability == 7), KLpos, KLgrad, KLidx);


  img = im*0;
  for i=1:size(KLidx,1)
    if KLref(i)
     img(KLpos(i,1), KLpos(i,2))=1;
    else
     img(KLpos(i,1),KLpos(i,2))=2;
    end
  end
  save_img(besos(img', 0, 3), frame_id, 1);
if conf.visualize_edges
  figure(1);
  imagesc(img);axis equal;colorbar;
  title('edges joined & rejected')
end

%% VISUALS %%
save_img(besos(edge_probability', 0, 7), frame_id, 2);
if conf.visualize_edges
  [y x] = meshgrid(1:size(dog,2), 1:size(dog,1));
  figure(2);
  imagesc(edge_probability);
  axis equal; colormap jet; colorbar;
  hold on;
  quiver(ys_im+y, xs_im+x, vec_y, vec_x);
  title('edge probability')
end

keylines = zeros([size(im),3], 'uint8');
for idx = 1:KLctr
  color = randi([48 255], [1 3]);
  if KLidx(idx,1) ~= 0
    continue
  end
  keylines( KLpos(idx,1), KLpos(idx,2), :) = color;
  next = KLidx(idx,2);
  while next ~= 0
    if keylines( KLpos(next,1), KLpos(next,2), 1) ~= 0 % needed because something is no yes
      break
    end  
    keylines( KLpos(next,1), KLpos(next,2), :) = color;
    next = KLidx(next,2);
  end  
end

%% MOVE TO STRUCT %%
KL = struct();
KL.frame_id = frame_id;
KL.ctr = KLctr;
KL.pos = KLpos(:,end:-1:1);
KL.posSubpix = KLposSubpix(:,end:-1:1);
KL.posImage = KLposImage(:,end:-1:1);
KL.posImageMatch = KLposImageMatch(:,end:-1:1);
KL.idx = KLidx;
KL.grad = KLgrad(:,end:-1:1);
KL.vers = KLvers(:,end:-1:1);
KL.norm = KLnorm;
KL.rho = KLrho;
KL.rhoPredict = KLrhoPredict;
KL.matching = KLmatching;
KL.forward = KLforward;
KL.frames = KLframes;
KL.matchedGrad = KLmatchedGrad(:,end:-1:1);
KL.matchedNorm = KLmatchedNorm;

save_img(keylines, frame_id, 3);
if conf.visualize_edges
  figure(3);
  imshow(keylines);
  hold on; quiver(ys_im+y, xs_im+x, vec_y, vec_x);
  title('edge joining')
end

if conf.visualize_3D && use_depth
  figure(100);
  X = KL.posImage(:,1) ./ conf.zf ./ KL.rho(:,1);
  Y = KL.posImage(:,2) ./ conf.zf ./ KL.rho(:,1);
  Z = 1 ./ KL.rho(:,1);
  hold on
  plot3(X,Z,Y,'b.')
  for yter = 1:size(edge_probability,1)
    for xter = 1:size(edge_probability,2)
      if edge_probability(yter,xter) == 6
        if depth(yter,xter) == 0
          col = 'r.';
          d = conf.cheat_lower_bound;
        else
          col = 'k.';
          d = depth(yter,xter);
        end  
        XY = pixelToNormalized([xter yter]) ./ conf.zf * d;
        plot3(XY(1), d, XY(2), col);
      end
    end
  end  
  set(gca,'zdir','reverse')
  hold off
end

l_kl_num = KL.ctr;
img_mask = img_mask'; %to change mask to xyz
