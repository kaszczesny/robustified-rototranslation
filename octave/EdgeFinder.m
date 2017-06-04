%% SETUP %%
clear
pkg load image
setup_opencv
graphics_toolkit('fltk')


%% CONFIG %%
Config


%% DOG %%
im = imresize(imread(im_name), scale);
im_blurred1 = double(cv.GaussianBlur(
              im, "KSize", [ksize,ksize], 
              "SigmaX", sigma1, "SigmaY", sigma1));
im_blurred2 = double(cv.GaussianBlur(
              im, "KSize", [ksize,ksize],
              "SigmaX", sigma2, "SigmaY", sigma2));
              %% ratio of kernels should be 4:1 or 5:1
dog = double(im_blurred2) - double(im_blurred1);


%% KEYLINE INIT %%
KLctr = 0;
KLpos = []; % pixel position
KLposSubpix = []; % subpixel position
KLidx = []; % index of previous/next keyline
KLgrad = []; % local third derivative vector
KLrho = []; % estimated inverse depth


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
    if ~test_1(n2gI(yter,xter))
      edge_probability(yter,xter) = 1;
      continue
    end
    
    % Test 2: current point is (more or less) in the middle of a keyline
    Y = dog(yter+[-win_s:win_s], xter+[-win_s:win_s]);
    pn = sum(sign(Y));
    if abs(pn) > PosNegThresh * (win_s*2+1).^2
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
    
    xs_art(yter,xter) = xs;
    ys_art(yter,xter) = ys;
    
    
    if visualize
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
    n = n ./ sqrt(sum(n([1 2]).^2));
    vec_x(yter,xter) = n(2); %weirdly, n(1) and n(2) are swapped
    vec_y(yter,xter) = n(1);
    
    edge_probability(yter,xter) = 5;
  
    KLctr += 1;
    KLpos = [KLpos; yter, xter];
    KLposSubpix = [KLposSubpix; ys+yter, xs+xter];
    KLidx = [KLidx; 0, 0];
    KLgrad = [KLgrad; theta(1:2)'];
    KLrho = [KLrho, 0, 0];
  end
end


%% KEYLINE JOINING %%
KLidx = KeylineJoiner((edge_probability == 5), KLpos, KLgrad, KLidx);


%% VISUALS %%
figure; imagesc(edge_probability); axis equal; colormap jet; colorbar;
hold on; quiver(vec_y, vec_x);

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
figure; imshow(keylines); 