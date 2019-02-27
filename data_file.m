%=========================================================================% 
% data_file.m initializes the output file where the state variables are   %
% written during the simulation.                                          %
%=========================================================================%

classdef data_file < handle
    properties
        state_vars      % state variables
        delimiter       % delimiter type
        header          % integer value of header lines
    end
    
    methods
        function obj = data_file(f1,d,h)
            obj.state_vars = f1;
            obj.delimiter = d;
            obj.header = h;
        end
    end
end