close all

setup_opencv
pkg load image
conf = Config();
load("../../points.mat");
p1 = first;
p2 = second;

K = [conf.zf  0       conf.principal_point(2);
     0        conf.zf conf.principal_point(1);
     0        0       1];
E = cv.findEssentialMat(p1, p2, 'CameraMatrix', K );

S = cv.decomposeEssentialMat(E);
R1 = S.R1;
R2 = S.R2;
T = S.t;

im1 = imresize(imread("../data/TUM/000015.png"), 1);
im2 = imresize(imread("../data/TUM/000008.png"), 1);
g1 = double(rgb2gray(im1));
g2 = double(rgb2gray(im2));

stereo = cv.StereoBM;
disparity = stereo.compute(g1, g2) /4;
imagesc(double(disparity));