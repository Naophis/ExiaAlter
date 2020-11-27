clear;

home = pwd;
script_file = mfilename('fullpath');
[filepath, name, ext] = fileparts(script_file);
cd(filepath);

Simulink.importExternalCTypes('../include/bus.h');

dt = 0.001/4;

test_mode = 0; % straight
test_mode = 1; % slalom
% test_mode = 2; % pivot
% test_mode = 3; % back_straight

%default
sla.base_alpha = 0;
sla.base_time = 0;
sla.limit_time_count = 0;
sla.pow_n = 0;
sla.state = 0;
sla.counter = int32(0);

if test_mode == 0
    % tgt
    v_max = 5000;
    end_v = 0;
    accl = 20000;
    decel = -20000;
    w_max = 0;
    end_w = 0;
    param_alpha = 0;
    tgt_dist = 180 * 16;
    tgt_angle = 0;

    % ego
    ego_v = 0;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

    PhyBase.accl_limitter_x = [0, 2500, 3500, 4500, 6500];
    PhyBase.accl_limitter_gain = [1, 1, 0.8, 0.6, 0.2];

    radius = 120;
    alphaTemp = (ego_v / radius);
    sla.base_alpha = alphaTemp;
    sla.base_time = 0.08233642578125;
    sla.limit_time_count = sla.base_time * 2 / dt;
    sla.pow_n = 4;
    sla.state = 0;
    sla.counter = int32(1);

end

if test_mode == 1
    % tgt
    v_max = 1500;
    end_v = v_max;
    accl = 28000;
    decel = -20000;
    w_max = 0;
    end_w = 0;
    param_alpha = 0;
    tgt_dist = 180 * 1800;
    tgt_angle = 0;

    % ego
    ego_v = v_max;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

    PhyBase.accl_limitter_x = [0, 2500, 3500, 4500, 6500];
    PhyBase.accl_limitter_gain = [1, 1, 0.8, 0.6, 0.2];

    radius = 120;
    alphaTemp = (ego_v / radius);
    sla.base_alpha = alphaTemp;
    sla.base_time = 0.08233642578125;
    sla.limit_time_count = sla.base_time * 2 / dt;
    sla.pow_n = 4;
    sla.state = 0;
    sla.counter = int32(0);

end

if test_mode == 2

    % tgt
    v_max = 0;
    end_v = 0;
    accl = 0;
    decel = 0;
    tgt_angle = 90 * 2 * pi / 360;

    param_alpha = -10;
    w_max = 2.5;
    end_w = 0.1;
    tgt_dist = 0;

    if param_alpha < 0
        w_max = w_max * (-1);
        tgt_angle = tgt_angle * (-1);
        end_w = end_w * (-1);
    end

    % ego
    ego_v = 0;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

end

if test_mode == 3
    % tgt
    v_max = -100;
    end_v = 0;
    accl = -1000;
    decel = 500;
    w_max = 0;
    end_w = 0;
    param_alpha = 0;
    tgt_dist = -180;
    tgt_angle = 0;

    % ego
    ego_v = 0;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

    PhyBase.accl_limitter_x = [0, 2500, 3500, 4500, 6500];
    PhyBase.accl_limitter_gain = [1, 1, 0.8, 0.6, 0.2];

    radius = 120;
    alphaTemp = (ego_v / radius);
    sla.base_alpha = alphaTemp;
    sla.base_time = 0.08233642578125;
    sla.limit_time_count = sla.base_time * 2 / dt;
    sla.pow_n = 4;
    sla.state = 0;
    sla.counter = int32(1);

end

predict_list = [1:10];

cd(home);
