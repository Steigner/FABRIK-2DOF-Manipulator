function [A] = FK(th)
    % FK - Forward Kinematics - nonmodified table
    % :param: 
    %   th - thn: double array - radians  
    % :returns: 
    %   DH table: double matrix

    % call DH-Parameters from ./DH_params function  
    % alfa, translation, length
    [o,d,a] = DH_params;
    
    for i=1:length(th)
        % D-H Table
        A_x = [
            cos(th(i)),  -sin(th(i)) * cos(o(i)),     sin(th(i)) * sin(o(i)),     a(i) * cos(th(i)) ;
            sin(th(i)),   cos(th(i)) * cos(o(i)),    -cos(th(i)) * sin(o(i)),     a(i) * sin(th(i)) ;
            0         ,   sin(o(i))             ,     cos(o(i))             ,     d(i)              ;
            0         ,   0                     ,     0                     ,     1          
        ];
        if i == 1
            A = A_x;
        else
            A = A * A_x;
        end
    end
end