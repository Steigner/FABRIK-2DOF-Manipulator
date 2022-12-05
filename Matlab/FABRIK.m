function [p] = FABRIK(p, n, t, tol)
    % FABRIK - Forward And Backward Reaching Inverse Kinematics
    
    % the distances between each joint 
    d = zeros(1,n);
    for i=1:n-1
        d(i) = norm(p(i+1,:) - p(i,:), "fro");
    end
    % The distance between root and target
    dist = norm(p(1,:) - t, "fro");

    % % Check whether the target is within reach
    if dist > sum(d)
        disp('[INFO] Target is unreachable!')
        for i=1:n-1
            % Find the distance ri between the target t and the joint
            r = norm(t - p(i,:), "fro");
            lambda_ = d(i) / r;
            % Find the new joint positions pi
            p(i+1,:) = (1-lambda_) * p(i,:) + lambda_ * t;
        end

    else
        disp('[INFO] Target is reachable!')
        %  thus, set as b the initial position of the joint p1
        b = p(1,:);
        difa = norm(p(n,:) - t, "fro");
        % Check whether the distance between the end effector pn
        % and the target t is greater than a tolerance.
        while difa > tol
            % STAGE 1: FORWARD REACHING
            % Set the end effector pn as target t
            p(n,:) = t;
            for i=n-1:-1:1
                % Find the distance ri between the new joint position
                % pi+1 and the joint pi
                r = norm(p(i+1,:) - p(i,:), "fro");
                lambda_ = d(i) / r;
                % Find the new joint positions pi.
                p(i,:) = (1 - lambda_) * p(i+1,:) + lambda_ * p(i,:);
            end
    
            % STAGE 2: BACKWARD REACHING
            % Set the root p1 its initial position.
            p(1,:) = b;
            for i=1:n-1
                % % Find the distance ri between the new joint
                % position pi
                r = norm(p(i+1,:) - p(i,:), "fro");
                lambda_ = d(i) / r;
                % Find the new joint positions pi.
                p(i+1,:) = (1 - lambda_) * p(i,:) + lambda_ * p(i+1,:);
            end
            difa = norm(p(n,:) - t);
        end
    end
end