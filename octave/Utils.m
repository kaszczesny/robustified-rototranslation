1;

function coords = pixelToNormalized(coords)
% convert (sub)pixel coordinates (x,y) of size n,2 to normalized coordinates

  global conf;

  coords -= conf.principal_point; % move principal point to 0,0
  %coords = coords ./ conf.zf; % normalize to [-1 1]
end

function coords = normalizedToPixel(coords)
% convert normalized coordinates (y,x) of size n,2 to (sub)pixel coordinates

  global conf;
  
  %coords = coords .* conf.zf;
  coords += conf.principal_point;
end

function [colored] = besos(img, lower, upper)
    % https://www.mathworks.com/matlabcentral/answers/45348-convert-matrix-to-rgb
    N = 128;
    colors = jet(N);
    
    img = img';
    
    if nargin == 3
        img( img < lower ) = lower;
        img( img > upper ) = upper;
    else
        lower = min(img(:));    
        upper = max(img(:));    
    end
    
    space = linspace(lower, upper, N);
    Gs = interp1(space, 1:N, img);
    Gs = round(Gs);
    
    colored = reshape(colors(Gs,:), [size(Gs) 3]);
end

function [] = save_img(img, figure_num)
  global conf;
  global frame;
  
  if ~conf.save_images
    return
  end  
  
  imwrite(img, sprintf("%s/%02d_%04d.png", conf.output_folder, ...
    figure_num, frame));
end
