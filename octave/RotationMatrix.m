function R = RotationMatrix(w)
  % https://pixhawk.org/_media/dev/know-how/jlblanco2010geometry3d_techrep.pdf
  % https://github.com/edrosten/TooN/blob/master/so3.h#L254
  % w is a vector that defines rotation axis; it's length is the rotation angle
  
  skewSymmetric = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
  R = expm(skewSymmetric);
  return 
end