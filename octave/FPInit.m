clear all
close all

pkg load image
global conf = Config();
setup_opencv

%%%DETECTING%%%

max_corners = 50;

im1 = imresize(imread("../../00/image_0/000031.png"), 1);
im2 = imresize(imread("../../00/image_0/000032.png"), 1);
imsize = size(im1);

%im1 = im1(:, 300:end-300);
%im2 = im2(:, 300:end-300);

detector = cv.FeatureDetector('GFTTDetector', 'MaxFeatures', max_corners);
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

%%%MATCHING%%%

matcher = cv.DescriptorMatcher('BFMatcher', 'NormType', 'Hamming', 'CrossCheck', 1);
matcher.add(des1);
matcher.train();
matches = matcher.match(des2);

imsize = size(im1);
dist_thresh = 0.25 * (imsize(1)^2 + imsize(2)^2)^(1/2)

figure()
im_plot = [im1, im2; im2, zeros(imsize)];
imshow(im_plot);
hold on;

for iter = 1:size(matches, 2)
  if matches(iter).distance < 20
    %matches(iter).distance
  
    color = rand(1,3);
    idx1 = matches(iter).queryIdx + 1; %c++ to matlab matrix indexing
    idx2 = matches(iter).trainIdx + 1; %
    kp1_ = kp1(idx1).pt;
    kp2_ = kp2(idx2).pt;
    
  %if (kp1_(1)*kp2_(1) + kp1_(2)*kp2_(2))^(1/2) < dist_thresh
    plot([kp1_(1), kp2_(1)+imsize(2)], ...
          [kp1_(2), kp2_(2)], "-o", ...
          'Color', color, 'markerfacecolor', color);
    plot([kp1_(1), kp2_(1)], ...
          [kp1_(2), kp2_(2)+imsize(1)], "-o", ...
          'Color', color, 'markerfacecolor', color);
    %(kp1_(1)*kp2_(1) + kp1_(2)*kp2_(2))^(1/2)
  end
end

hold off;
%figure()
%out = cv.drawMatches(im1, kp1, im2, kp2, matches);
%imshow(out)