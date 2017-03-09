#!/usr/local/bin/octave -f

run('../../octave/setup_opencv.m')

im = cv.imread("../../tex/img/agh.jpg");

im = cv.cvtColor( im, 'RGB2GRAY' );

im2 = cv.cornerHarris(im);

im3 = cv.normalize( im2, 'NormType', 'MinMax', 'DType', 'double', 'Alpha', 0, 'Beta', 255 );
im3 = cv.convertScaleAbs( im3 );

cv.imwrite('../../data/dummy_test2.png', im3)
