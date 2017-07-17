res = 1000;
im_size = [res, 2*res];

%white bgr
im = ones(im_size);

%gray rectangles
im(res/4+1:res*3/4,res/2+1:res) = ones(res/2,res/2)*1/5;
im(res/4+1:res*3/4,res+1:res*3/2) = ones(res/2,res/2)*4/5;

%black border
border_width = res*0.1;
im_final = zeros(im_size + 2*border_width);
im_final( border_width+1:res+border_width, ...
          border_width+1:2*res+border_width ) = im;

imshow(im_final)