function [im1, im2] = Generate2DData (imname, ytrans, xtrans, roll, outliers)
%im - name of image to transform
%ytrans(pixel), xtrans(pixel), roll(angle) - transformation
%outliers - 1 to add outliers, 0 otherwise
close all
if ~exist('OPENCV')
  pkg load image
  setup_opencv
  graphics_toolkit('fltk')
  OPENCV = 1;
end

im = imread(imname);
im = double(rgb2gray(im));
imsizetotal_ = size(im);
imsize_ = [300, 400];

start_ = floor(imsizetotal_-imsize_)/2;

im1 = im( start_(1)+1:start_(1)+imsize_(1) , start_(2)+1:start_(2)+imsize_(2) );

preim2 = imrotate(im, -roll, 'bilinear', 'loose');
imsizetotal_ = size(preim2);
im2 = zeros( imsizetotal_ + abs([ytrans, xtrans]) );
if ytrans < 0
  if xtrans > 0
    im2( abs(ytrans)+1:end, abs(xtrans)+1:end ) = preim2;
  else
    im2( abs(ytrans)+1:end, 1:imsizetotal_(2) ) = preim2;
  end
else
  if xtrans > 0
    im2( 1:imsizetotal_(1), abs(xtrans)+1:end ) = preim2;
  else
    im2( 1:imsizetotal_(1), 1:imsizetotal_(2) ) = preim2;
  end
end

imsizetotal_ = size(im2);
start_ = floor(imsizetotal_- imsize_)./2;
im2 = im2( start_(1)+1:start_(1)+imsize_(1) , start_(2)+1:start_(2)+imsize_(2) );

if outliers == 1
  maxmov = 11; % >10
  segment = 100;
  rstart = [randi(imsize_(1)-segment-2*maxmov+1), randi(imsize_(2)-segment-2*maxmov+1)]+maxmov;
  mov_ = randi(maxmov-10,1,2)+10;
  dir_ = randi(2,1,2);
  frag_ = im2(rstart(1):rstart(1) +segment-1, rstart(2):rstart(2) +segment-1);
  
  if dir_(1)
    if dir_(2)
      start_ = rstart - mov_;
    else
      start_ = [rstart(1) - mov_(1), rstart(2) + mov_(2)];  
    end
  else
    if dir_(2)
      start_ = [rstart(1) + mov_(1), rstart(2) - mov_(2)];
    else
      start_ = rstart + mov_;
    end
  end
  im2(start_(1):start_(1) +segment-1, start_(2):start_(2) +segment-1) = frag_;
end

axis equal;
subplot(211)
imagesc(im1)
axis equal; colormap gray;
subplot(212)
imagesc(im2)
axis equal; colormap gray;
end
