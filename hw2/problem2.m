%% Design Nonlinear MPC Controller
nlobj = nlmpc(4,4,2);
nlobj.Model.StateFcn = "manipulatorWithDisturbStateFcn";
nlobj.Ts = 0.1;
nlobj.Weights.OutputVariables = [2 1 0 0];
nlobj.Weights.ManipulatedVariablesRate = [0 0];
nlobj.Passivity.EnforceConstraint = true;
nlobj.Passivity.InputFcn = "getPassivityInput";
nlobj.Passivity.OutputFcn = "getPassivityOutput";

%% Closed-Loop Simulation
x0 = [-2;-1;1;1];
mdl = "manipulatorNLMPC";
open_system(mdl)

sim(mdl);
% 
open_system(mdl + "/Manipulator/states")