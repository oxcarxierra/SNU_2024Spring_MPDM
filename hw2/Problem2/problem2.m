clear; close all;
%% Define model and mpc
nlobj = nlmpc(4,4,2);
nlobj.Model.StateFcn = "manipulatorStateFcn";
nlobj.Ts = 0.1;
nlobj.Weights.OutputVariables = [2 1 0 0];
nlobj.Weights.ManipulatedVariablesRate = [0 0];

nlobj.Passivity.EnforceConstraint = true;
nlobj.Passivity.InputFcn = "getPassivityInput";
nlobj.Passivity.OutputFcn = "getPassivityOutput";

%% Initial State
x0 = [-2;-1;1;1];

%% Open NLMPC
mdl = "manipulatorNLMPC";
open_system(mdl)

%% Run NLMPC
sim(mdl);
open_system(mdl + "/Manipulator/states")
%% 
% nlobj.Passivity.EnforceConstraint = false;
% sim(mdl);
