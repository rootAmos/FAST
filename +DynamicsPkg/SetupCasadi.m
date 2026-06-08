function [] = SetupCasadi()
%
% [] = SetupCasadi()
%
% Add CasADi to the MATLAB path when it is not already available.
%

try
    import casadi.*
    Opti(); %#ok<VUNUS>
    return
catch
end

CandidatePaths = string(getenv("CASADI_MATLAB_PATH"));
CandidatePaths(end + 1) = "C:\Program Files\MATLAB\casadi-3.7.2-windows64-matlab2018b";
for ipath = 1:length(CandidatePaths)
    if strlength(CandidatePaths(ipath)) > 0 && exist(CandidatePaths(ipath), "dir") == 7
        addpath(CandidatePaths(ipath));
        break
    end
end

try
    import casadi.*
    Opti(); %#ok<VUNUS>
catch Error
    error("ERROR - DynamicsPkg.SetupCasadi: CasADi is not on the MATLAB path. " + ...
        "Install CasADi or set CASADI_MATLAB_PATH to its MATLAB folder.\n%s", Error.message);
end

end
