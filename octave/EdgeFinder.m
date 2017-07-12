function [KL] = EdgeFinder(im_name)

persistent frame_id = 0;
frame_id++;

%% CONFIG %%
conf = Config();
win_s = conf.win_s;

%% DOG %%
im = imresize(imread(im_name), conf.scale);
im_blurred1 = double(cv.GaussianBlur(
              im, "KSize", [conf.ksize,conf.ksize], 
              "SigmaX", conf.sigma1, "SigmaY", conf.sigma1));
im_blurred2 = double(cv.GaussianBlur(
              im, "KSize", [conf.ksize,conf.ksize],
              "SigmaX", conf.sigma2, "SigmaY", conf.sigma2));
              %% ratio of kernels should be 4:1 or 5:1
dog = double(im_blurred2) - double(im_blurred1);


%% KEYLINE INIT %%
%{
m_m     KLgrad (2d f)
u_m     normalized KLgrad (2d f)
n_m     norm of KLgrad (f)

c_p     KLposSubpix (2d f)
round(c_p)  KLpos

rho     Estimated Inverse Depth (f)
s_rho   Estimated Inverse Depth Uncertainty (f)
        KLrho

rho0    Predicted Inverse Depth in EKF (use only if rescaling) (f)
s_rho0  Predicted Inverse Depth Uncertainty (f)
        KLrhoPredict

p_m     Subpixel position in homo coordinates (plane on focal length zf) (2d f)
        KLposImage
p_m_0   matched KL (subpixel) position in homo coordinates (2d f)
        #######

m_id    Id of the matching KL
        #######
m_id_f  Id of the matching KL by forward matching
        KLforward

m_num   Number of consecutive matches (int)
        KLframes

m_m0    Gradient of matched KL (2d f)
        ######
n_m0    Norm of m_m0 (f)

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
KLidx = []; % index of previous/next keyline
KLgrad = []; % local third derivative vector
KLrho = []; % estimated inverse depth and inverse depth uncertainty
KLrhoPredict = []; % predicted inverse depth and inverse depth uncertainty
KLforward = []; % id of forward match
KLframes = []; % number of consecutive matches


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
n2gI = sqrt(img_dx.^2 + img_dy.^2);

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
    if ~conf.test_1(n2gI(yter,xter))
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
    
    
    if conf.visualize && 0
      disp([yter xter])
      VisualizeZeroCrossing(Y, theta, im_blurred2(yter+[-win_s:win_s], xter+[-win_s:win_s]))
      input('...');
      close;
    end
    
    if max(abs([ys xs])) > 0.5;
      edge_probability(yter,xter) = 4;
      continue
    end
    
    % local normal vector, normalized to versor after projection to z=0
    n = [-theta(1) -theta(2) 1];
    %n = n ./ sqrt(sum(n([1 2]).^2));
    vec_x(yter,xter) = n(2); %weirdly, n(1) and n(2) are swapped
    vec_y(yter,xter) = n(1);
    
    edge_probability(yter,xter) = 5;
  
    KLctr += 1;
    KLpos = [KLpos; yter, xter];
    KLposSubpix = [KLposSubpix; ys+yter, xs+xter];
    KLposImage = [KLposImage; [ys+yter, xs+xter]-conf.principal_point];
    KLidx = [KLidx; 0, 0];
    KLgrad = [KLgrad; theta([2 1])'];
    KLrho = [KLrho; conf.RHO_INIT, conf.RHO_MAX];
    KLrhoPredict = [KLrhoPredict; 0, 0];
    KLforward = [KLforward; -1];
    KLframes = [KLframes; 0];
  end
end


%% KEYLINE JOINING %%
[KLidx, KLref] = KeylineJoiner((edge_probability == 5), KLpos, KLgrad, KLidx);

if conf.visualize
  img = im*0;
  for i=1:length(KLidx)
    if KLref(i)
     img(KLpos(i,1), KLpos(i,2))=1;
    else
     img(KLpos(i,1),KLpos(i,2))=2;
    end
  end
  figure;imagesc(img);axis equal;
end

%% VISUALS %%
if conf.visualize
  [y x] = meshgrid(1:size(dog,2), 1:size(dog,1));
  figure;
  imagesc(edge_probability);
  axis equal; colormap jet; colorbar;
  hold on;
  quiver(ys_im+y, xs_im+x, vec_y, vec_x);
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
KL.pos = KLpos;
KL.posSubpix = KLposSubpix;
KL.posImage = KLposImage;
KL.idx = KLidx;
KL.grad = KLgrad;
KL.rho = KLrho;
KL.rhoPredict = KLrhoPredict;
KL.forward = KLforward;
KL.frames = KLframes;

if conf.visualize
  figure; imshow(keylines);
  hold on; quiver(ys_im+y, xs_im+x, vec_y, vec_x);
end

