clear all
close all

if ~exist('OPENCV')
  pkg load image
  pkg load optim
  pkg load quaternion
  setup_opencv
  graphics_toolkit('fltk')
  OPENCV = 1;
end


load prez_traj.mat

FitTrajectory;

FitTrajectory_(traj1_truth, traj1_est);
figure(1)
title('TUM fr3 teddy', 'fontsize', 20)

copied_legend = findobj(gcf(),"type","axes","Tag","legend");
set(copied_legend, "FontSize", 16);

FitTrajectory_(traj2_truth, traj2_est);
figure(2)
title('TUM fr2 desk with person', 'fontsize', 20)

copied_legend = findobj(gcf(),"type","axes","Tag","legend");
set(copied_legend, "FontSize", 16);