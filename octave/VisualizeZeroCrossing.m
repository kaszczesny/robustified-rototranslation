function [] = VisualizeZeroCrossing(nei, theta, im)
% @param nei - DoG neighbourhood (square matrix)
% @param theta - solution of the equation system ( z = theta(1)*x + theta(2)*y + theta(3) )
% @param im - original (smoothed) image in this neighbourhood (square matrix)

win_s = (size(nei,1) - 1)/2;

[x, y]= meshgrid([-win_s:win_s]);

figure
subplot(1,2,1)
mesh(y, x, nei) % DoG
xlabel('y')
ylabel('x')
zlabel('z')
hold on


z = theta(1)*x + theta(2)*y + theta(3); % approximated DoG
surf(y, x, z)
surf(y, x, x*0) % the zero plane

%{
  finding the zero-crossing:

  z = theta(1)*x + theta(2)*y + theta(3) and z = 0
  theta(2)*y = -theta(1)*x - theta(3)
  y = -theta(1)/theta(2)*x - theta(3)/theta(2) % L1: line passing through both planes
  
  y = theta(2)/theta(1)*x % L2: line on plane z=0 that is perpendicular to L1 and passes through (0,0)
  
  % zero-crossing is a point of intersection of L1 and L2
  
  y = -theta(1)/theta(2)*x - theta(3)/theta(2) and y = theta(2)/theta(1)*x
  [theta(2)/theta(1) + theta(1)/theta(2)]*x = -theta(3)/theta(2) | * theta(1)*theta(2)
  (theta(2).^2 + theta(1).^2)*x = -theta(1)*theta(3)
  x =                       -theta(1)*theta(3)/[theta(1)^2 + theta(2)^2]
  y = theta(2)/theta(1)*x = -theta(2)*theta(3)/[theta(1)^2 + theta(2)^2]
%}

xx = [-win_s win_s];
a = -theta(1)/theta(2);
b = -theta(3)/theta(2);
plot3( a*xx+b, xx, xx*0, 'bo-', 'linewidth', 5 ) % L1: line of crossing
plot3( xx, xx*(-a), xx*0, 'rs-', 'linewidth', 5) % L2: line perpendicular to L1 that passes through 0
disp([a b])

% check if further computations are ill-conditioned
if min(abs([theta(1), theta(2)])) < 1e-6
  title('bad conditioning')
  return 
end

% zero-crossing formulas
xs = -theta(1)*theta(3) / (theta(1)^2 + theta(2)^2);
ys = -theta(2)*theta(3) / (theta(1)^2 + theta(2)^2);
plot3( ys, xs, 0, 'gd', 'linewidth', 5) % zero-crossing

xlim([-win_s-1 win_s+1])
ylim([-win_s-1 win_s+1])

% normal vector
p0 = [0 0 theta(3)];
n = [-theta(1) -theta(2) 1];
n = n ./ sqrt(sum(n([1 2]).^2));
plotvec = @(p0, n, fmt) plot3( [p0(2)+[0 n(2)]], [p0(1)+[0 n(1)]], [p0(3)+[0 n(3)]], fmt, 'linewidth', 5);
plotvec(p0, n, 'm-');
plotvec([0 0 0], n - ([0 0 1]*n(3)), 'c-')

subplot(1,2,2)
mesh(y, x, im) % local pixel intensities
xlabel('y')
ylabel('x')
zlabel('z')
hold on

surf( a*[-win_s win_s; -win_s win_s]+b, [-win_s win_s; -win_s win_s], [min(im(:)).*[1 1]; max(im(:)).*[1 1]]) % L1: line of crossing 
plot3( [ys ys], [xs xs], [min(im(:))-5 max(im(:))+5], 'ko-', 'linewidth', 5) % good zero-crossing
xlim([-win_s-1 win_s+1])
ylim([-win_s-1 win_s+1])