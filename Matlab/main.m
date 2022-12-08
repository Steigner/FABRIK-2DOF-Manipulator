%% Compute FABRIK 2DOF SCARA robot
clear;
clc;

% You can simply change type of SCARA robot for demonstration -> DH params
name = 'ABB IRB 910SC (SCARA)';

% call DH-Parameters from ./DH_params function  
[~,~,a] = DH_params;

% set up tolerance
tolerance = 0.001;

% base rotation, arm1 rotation, arm2 rotation
q = [0, -13.0570, 147.1469];
q = deg2rad(q);

% length of input theta
len = length(q);

% set up goald
goal = [0, -0.2];

% range of SCARA robot
ax_wr = [-140.0, 140.0; -150.0, 150.0];

% choose if you want show workspace / simple animation of trajectory 
show_workspace = true;
show_origin_position = true;
show_anim = true;

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

while 1
    % from FK by DH matrix compute points of SCARA robot
    p = zeros(len,2);
    for i=1:len
        % call Forward-Kinematics from ./FK function  
        A = FK(q(1:i));
        p(i,:) = A(1:2,4)';
    end
    
    disp("[INFO] Start Computation")
    tic; 
    % call FABRIK compute from ./FABRIK function  
    p_ = FABRIK(p, len, goal, tolerance);
    fprintf("[INFO] End Computation - time: %.4f \n", toc);
    
    % compute result angle 2DOF SCARA robot
    th1 = atan2(p_(2,2) - p_(1,2), p_(2,1) - p_(1,1));
    th1 = - q(1) + th1;
    th2 = atan2(p_(3,2) - p_(2,2), p_(3,1) - p_(2,1));
    th2 = - q(1) - th1 + th2;
    
    % radians -> degree
    th1_deg = rad2deg(th1);
    th2_deg = rad2deg(th2);
    
    fprintf("[INFO] Theta1: %.2f \n", th1_deg);
    fprintf("[INFO] Theta2: %.2f \n", th2_deg);
    
    % set up x and y limits of plot
    xlim([-0.7 0.7]);
    ylim([-0.7 0.7]);
    hold on;
    grid on;
    % title of plot
    title(name);
    xlabel('x');
    ylabel('y');
    
    % plot proccesed positions
    h1 = plot( ...
        p_(:,1), ...
        p_(:,2), ...
        'LineWidth', 3, ...
        'Color', 'red', ...
        'Marker', 'o', ...
        'LineStyle', '-' ...
    );

    % plot goal
    h2 = plot( ...
        goal(1,1), ...
        goal(1,2), ...
        'LineWidth', 5, ...
        'Color', 'blue', ...
        'Marker', 'x', ...
        'MarkerSize', 10 ...
    );

    legend('Processed position', 'Goal');

    if show_origin_position
        % Color, RGB + Transparency
        h3 = plot( ...
            p(:,1), ...
            p(:,2), ...
            'LineWidth', 3, ...
            'Color', 'black', ...
            'Marker','o', ...
            'LineStyle','-' ...
        );
        legend('Processed position', 'Goal', 'Origin position');
    end  
    
    if show_anim
        % simple trajectory generation, defaultly 100 samples
        te1 = linspace(q(2), th1, 100);
        te2 = linspace(q(3), th2, 100);
        % pre-alocate array by zeros
        % 2 joint
        p_anim = zeros(length(te1),2);
        % 3 end effector / tool
        p2_anim = p_anim;
        
        % compute FK x,y points
        for i=1:length(te1)
            % call Forward-Kinematics from ./FK function  
            pom = FK([q(1), te1(i)]);
            p_anim(i,:) = pom(1:2,4)';
            pom = FK([q(1), te1(i), te2(i)]);
            p2_anim(i,:) = pom(1:2,4)';
        end
        
        % animate in for loop by length of generated samples
        for i=1:length(te1)
            h_anim = plot( ...
                [q(1), p_anim(i,1), p2_anim(i,1)], ...
                [q(1), p_anim(i,2), p2_anim(i,2)], ...
                'LineWidth', 3, ...
                'Color', 'blue', ...
                'Marker', 'o', ...
                'HandleVisibility', 'off' ...
            );
            % pause for 0.05 seconds
            pause(.05)
            % delete plot
            delete(h_anim) % <- comment if you want see all steps comment
        end
    end
    
    % change start angles of manipulator
    q = [q(1), th1, th2];

    % click input from plot
    try
        [x,y,~] = ginput(1);
        % coordinates from graph set as goal
        goal(1) = x;
        goal(2) = y;
        % delete plots for next computation
        delete(h1);
        delete(h2);
        if show_origin_position
            delete(h3);
        end
    catch
        return
    end
end