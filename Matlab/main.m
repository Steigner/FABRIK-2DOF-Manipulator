%% Compute FABRIK SCARA robot

clear;
clc;
clf;

% call DH-Parameters from ./DH_params function  
[~,~,a] = DH_params;

% set up tolerance
tolerance = 0.001;

% base rotation, arm1 rotation, arm2 rotation
q = [90, -13.0570, 147.1469];
q = deg2rad(q);

% set up goald
goal = [0, -0.2];

% range of SCARA robot
ax_wr = [-140.0, 140.0; -150.0, 150.0];

% from FK by DH matrix compute points of SCARA robot
len = length(q);
p = zeros(len,2);
for i=1:len
    % call Forward-Kinematics from ./FK function  
    A = FK(q(1:i));
    p(i,:) = A(1:2,4)';
end

disp("[INFO] Start Computation")
tic;

% call FABRIK compute 
p_ = FABRIK(p, len, goal, tolerance);
disp("[INFO] End Computation - time: " + string(toc));

% compute result angle 2DOF SCARA robot
th1 = atan2(p_(2,2) - p_(1,2), p_(2,1) - p_(1,1));
th1 = - q(1) + th1;
th2 = atan2(p_(3,2) - p_(2,2), p_(3,1) - p_(2,1));
th2 = - q(1) - th1 + th2;

% radians -> degree
th1_deg = rad2deg(th1);
th2_deg = rad2deg(th2);

disp("[INFO] Theta1: " + string(th1_deg));
disp("[INFO] Theta2: " + string(th2_deg));

%% Plot

% choose if you want show workspace / simple animation of trajectory 
show_workspace = true;
anim = true;

% set up x and y limits of plot
xlim([-0.7 0.7]);
ylim([-0.7 0.7]);
hold on;
grid on;
% title of plot
title('ABB IRB 910SC (SCARA)');
xlabel('x');
ylabel('y');

% simple workspace show
if show_workspace
    theta_1 = linspace(ax_wr(1), ax_wr(2), 100);
    theta_2 = linspace(ax_wr(3), ax_wr(4), 100);
    [theta_1_mg, theta_2_mg] = meshgrid(theta_1, theta_2);
    % !! change 
    x_p = a(2) * cos(theta_1_mg) + a(3) * cos(theta_1_mg + theta_2_mg);
    y_p = a(2) * sin(theta_1_mg) + a(3) * sin(theta_1_mg + theta_2_mg);
    scatter(x_p, y_p, 'green','MarkerFaceAlpha', .1, 'MarkerEdgeAlpha', .1, 'HandleVisibility', 'off')
end

plot(p(:,1), p(:,2), 'LineWidth', 3, 'Color', 'black', 'Marker','o' , 'LineStyle','-');
plot(p_(:,1), p_(:,2), 'LineWidth', 3, 'Color', 'red', 'Marker','o' , 'LineStyle','-');

% anim simple trajectory
if anim
    te1 = linspace(q(2), th1, 100);
    te2 = linspace(q(3), th2, 100);
    p_anim = zeros(length(te1),2);
    for i=1:length(te1)
        % call Forward-Kinematics from ./FK function  
        pom = FK([q(1), te1(i), te2(i)]);
        p_anim(i,:) = pom(1:2,4)';
    end
    for i=1:length(te1)
        plot(p_anim(i,1), p_anim(i,2), 'LineWidth', 3, 'Color', 'red', 'Marker', 'o', 'HandleVisibility', 'off');
        pause(.1)
    end
end

plot(goal(1,1),goal(1,2),'LineWidth', 5, 'Color', 'blue', 'Marker', 'x', 'MarkerSize', 10);
legend('Origin position', 'Processed position', 'Goal');