%Initialize path
base_path = fileparts(mfilename('fullpath'));
addpath(...
        genpath(fullfile(base_path, "UR5_example")), ...
        genpath(fullfile(base_path, "functions")) ...
);

     run('../matlab_toolbox/matlab_tools_path_init.m');
     run('../robotics-dep-ext/matlab_ext_path_init.m');
     run('../robotics-toolbox/robotics_toolbox_path_init.m');
     