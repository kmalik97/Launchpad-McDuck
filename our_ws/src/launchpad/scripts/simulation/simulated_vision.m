function [H, K, R, t] = simulated_vision()
%--------------------------------------------------------------------------
% determine homography matrix
%--------------------------------------------------------------------------
% camera intrinsic matrix
K = [97.19328308 0 161.60555983; 0 97.18055725 122.03446744; 0 0 1];

% camera pose
location = [0; 0; 100];
orientation = rot([1; 0; 0], deg2rad(110));

% get camera extrinsic parameters from camera pose
[R, t] = cameraPoseToExtrinsics(orientation, location);

% MATLAB uses different form than what we learned in class
R = R';
t = t';

% homography matrix
H = K * [R(:,1:2) t];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% simulated duckiebot vision
%--------------------------------------------------------------------------
N = 100;
zvec = zeros(1, N);

% yellow line params
% 10cm left of duckiebot
yellowOffset = 100;
x = linspace(-100, 50, N);
y = linspace(location(2), 500, N);

% desired heading params
% 12cm right of yellow line
laneOffset = 120;

% yellow line in the world plane
worldYellow = [x - yellowOffset; y; zvec + 1];

% white line in the world plane
worldWhiteRight = worldYellow + [2 * laneOffset; 0; 0];
worldWhiteLeft = worldYellow - [2 * laneOffset; 0; 0];

% desired heading in the world plane
worldDesired = worldYellow + [laneOffset; 0; 0];

% actual heading in the world plane
worldActual = [zvec; y; zvec+1];

% yellow line in the image plane
imageYellow = H * worldYellow;
imageYellow = imageYellow ./ imageYellow(3,:);

% white line in the image plane
imageWhiteRight = H * worldWhiteRight;
imageWhiteRight = imageWhiteRight ./ imageWhiteRight(3,:);
imageWhiteLeft = H * worldWhiteLeft;
imageWhiteLeft = imageWhiteLeft ./ imageWhiteLeft(3,:);

% desired heading in the world plane
imageDesired = H * worldDesired;
imageDesired = imageDesired ./ imageDesired(3,:);

% actual heading in the world plane (should be straight down the middle)
imageActual = H * worldActual;
imageActual = imageActual ./ imageActual(3,:);

% 1st degree fit of desired heading
p = polyfit(worldDesired(2,:), worldDesired(1,:), 1);

% operating point in the world plane
% 10cm in front of duckiebot
worldYOperating = 100;
worldXOperating = polyval(p, worldYOperating);

% operating point in the image plane
imageOperating = H * [worldXOperating; worldYOperating; 1];
imageXOperating = imageOperating(1) / imageOperating(3);
imageYOperating = imageOperating(2) / imageOperating(3);

% plot limits
lims = [-500 300 -100 500 0 200];

% 3D comparison
figure(1);
clf(1);
plotCamera('Location', location, 'Orientation', orientation, 'Size', 20);
grid on;
hold on;
plot3(worldYellow(1,:), worldYellow(2,:), zeros(1, N), 'y');
plot3(worldWhiteRight(1,:), worldWhiteRight(2,:), zeros(1, N), 'w');
plot3(worldWhiteLeft(1,:), worldWhiteLeft(2,:), zeros(1, N), 'w');
plot3(worldDesired(1,:), worldDesired(2,:), zeros(1, N), 'g');
plot3(worldActual(1,:), worldActual(2,:), zeros(1, N), 'r');
plot3(worldXOperating, worldYOperating, 0, 'cx', 'MarkerSize', 10);
set(gca,'GridColor', 'w')
set(gca,'Color', 'k')
axis(lims);
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
legend({'Yellow Line', 'Right White Line', 'Left White Line', 'Desired Heading', 'Actual Heading', 'Operating Point'}, 'TextColor', 'w');
title('3D Situation');
hold off;

% 2D vision comparison
figure(2);
clf(2);
subplot(1,2,1);
grid on;
hold on;
plot(worldYellow(1,:), worldYellow(2,:), 'y');
plot(worldWhiteRight(1,:), worldWhiteRight(2,:), 'w');
plot(worldWhiteLeft(1,:), worldWhiteLeft(2,:), 'w');
plot(worldDesired(1,:), worldDesired(2,:), 'g');
plot(worldActual(1,:), worldActual(2,:), 'r');
plot(worldXOperating, worldYOperating, 'cx', 'MarkerSize', 10);
set(gca,'GridColor', 'w')
set(gca,'Color', 'k')
axis(lims);
xlabel('X Distance (mm)');
ylabel('Y Distance (mm)');
legend({'Yellow Line', 'Right White Line', 'Left White Line', 'Desired Heading', 'Actual Heading', 'Operating Point'}, 'TextColor', 'w');
title('World Plane');
hold off;

subplot(1,2,2);
grid on;
hold on;
plot(imageYellow(1,:), imageYellow(2,:), 'y');
plot(imageWhiteRight(1,:), imageWhiteRight(2,:), 'w');
plot(imageWhiteLeft(1,:), imageWhiteLeft(2,:), 'w');
plot(imageDesired(1,:), imageDesired(2,:), 'g');
plot(imageActual(1,:), imageActual(2,:), 'r');
plot(imageXOperating, imageYOperating, 'cx', 'MarkerSize', 10);
set(gca,'GridColor', 'w')
set(gca,'Color', 'k')
set(gca, 'ydir', 'reverse');
axis(lims);
xlabel('Width (pixels)');
ylabel('Height (pixels)');
legend({'Yellow Line', 'Right White Line', 'Left White Line', 'Desired Heading', 'Actual Heading', 'Operating Point'}, 'TextColor', 'w');
title('Image Plane');
hold off;
%--------------------------------------------------------------------------
end