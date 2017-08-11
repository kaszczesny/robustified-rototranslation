clear -exclusive OPENCV
if ~exist('OPENCV')
  pkg load image
  setup_opencv
  graphics_toolkit('fltk')
  OPENCV = 1;
end  

pkg load optim

GlobalTracker;
conf = Config();

KL_prev = EdgeFinder(conf.im_name(1));
KL      = EdgeFinder(conf.im_name(2));

P0Im = zeros(KL_prev.ctr, 3); %image coordinates
P0m = zeros(KL_prev.ctr, 3); %3d coordinates

%converto to ltcv
P0Im(:,1) = KL_prev.posImage(:,1); % x
P0Im(:,2) = KL_prev.posImage(:,2); % y
P0Im(:,3) = KL_prev.rho(:,1);

%proyect (eq 4.) imgage -> 3d
P0m(:,3) = 1./ P0Im(:,3); % depth from inverse depth
Pz_zf = 1./conf.zf * P0m(:,3);
P0m(:,1) = P0Im(:,1) .* Pz_zf; %x
P0m(:,2) = P0Im(:,2) .* Pz_zf; %y

max_s_rho = EstimateQuantile(KL_prev);

[distance_field, KLidx_field] = AuxiliaryImage(KL);



y = @(x,pin) TryVelRot(0,0,pin,KL_prev, KL, P0m, max_s_rho, [], distance_field, KLidx_field,0,1);
y2 = @(pin) TryVelRot(0,0,pin,KL_prev, KL, P0m, max_s_rho, [], distance_field, KLidx_field,0,1);

%[~, x] = leasqr(KL_prev.posSubpix, KL_prev.frames*0, zeros(1,6),y)

fminsearch(y2, zeros(1,6))
