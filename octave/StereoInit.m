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

im1 = imresize(imread("../../00/image_0/000010.png"), 1);
im2 = imresize(imread("../../00/image_1/000010.png"), 1);

stereo = cv.StereoBM;
disparsity = stereo.compute(im1, im2) /16;
imagesc(double(disparsity))