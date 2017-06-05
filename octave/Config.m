im_name = '../data/000012_10.png';
%im_name = '../data/IMG_20170402_145917_1.jpg';
scale = 1/3;

  % DoG:
ksize = 13;
%sigma1 = 3; 
%sigma2 = 2;
sigma1 = 1.7818;
sigma2 = 2.30029;

  % Test 1:
thresh_grad = 0.015;
max_img_value = 255;
test_1 = @(n2gI) n2gI > (thresh_grad*max_img_value).^2; % n2gI was squared, but didn't work

  % Test 2:
win_s = 2;
PosNegThresh = 0.2;

visualize = 0;

% auxiliary image
max_r = 5;
