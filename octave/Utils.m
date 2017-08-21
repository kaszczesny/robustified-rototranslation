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

%todo: https://www.mathworks.com/matlabcentral/answers/45348-convert-matrix-to-rgb