function [alfa,d,a] = DH_params
    % D-H Parameters: ABB IRB 910SC (SCARA)
    % :params: None 
    % :returns: 
    %   alfa: double array
    %   d: double array
    %   a: double array
    
    % base, link1, link2
    % alfa
    alfa = [0.0, 0.0, 0.0];
    % translation
    d = [0.0, 0.0, 0.0];
    % arm length
    a = [0.0, 0.3, 0.25];
end