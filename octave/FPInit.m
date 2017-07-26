clear all
close all

conf = Config();
pkg load image
setup_opencv

%%%DETECTING%%%

max_corners = 200;

im1 = imresize(imread("../../00/image_0/000011.png"), 1);
im2 = imresize(imread("../../00/image_0/000012.png"), 1);

%nie przyjmuje takich keypointow to extractor.compute
%FP1 = cv.goodFeaturesToTrack(im1, 'MaxCorners', max_corners);
%FP2 = cv.goodFeaturesToTrack(im2, 'MaxCorners', max_corners);

%%%MATCHING%%%

detector = cv.FeatureDetector('ORB');
kp1 = detector.detect(im1);
kp2 = detector.detect(im2);

if size(kp1,2) > size(kp2,2)
  kp_t = kp1;
  kp1 = kp2;
  kp2 = kp_t;
end

extractor = cv.DescriptorExtractor('ORB');
des1 = extractor.compute(im1, kp1);
des2 = extractor.compute(im2, kp2);

matcher = cv.DescriptorMatcher('BFMatcher', 'NormType', 'L2');
%matcher.add(des1);
%matcher.train();
matches = matcher.match(des2, des1);

figure()
imsize = size(im1);
im_plot = [im1, im2; im2, zeros(imsize)];
imshow(im_plot);
hold on;

for iter = 1:size(kp1,2)
  color = rand(1,3);
  idx1 = matches(iter).queryIdx + 1; %c++ to matlab matrix indexing
  idx2 = matches(iter).trainIdx + 1; %
  kp1_ = kp1(idx1).pt;
  kp2_ = kp2(idx2).pt;
  
  
  plot([kp1_(1), kp2_(1)+imsize(2)], ...
        [kp1_(2), kp2_(2)], "-o", ...
        'Color', color, 'markerfacecolor', color);
  plot([kp1_(1), kp2_(1)], ...
        [kp1_(2), kp2_(2)+imsize(1)], "-o", ...
        'Color', color, 'markerfacecolor', color);
end
hold off;