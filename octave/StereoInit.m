conf.Config();

K = [conf.zf  0       conf.principal_point(2);
     0        conf.zf conf.principal_point(1);
     0        0       1];
E = cv.findEssentialMat(p1, p2, 'CameraMatrix', K, );

S = cv.decomposeEssentialMat(E);
R1 = S{}.R1;
R2 = S{}.R2;
T = S{}.T;

stereo = cv2.StereoBM;
disparsity = stereo.compute(p1, p2);