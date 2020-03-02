import csv

# body_vel_std = 0.01936
# body_acc_std = 0.7101
# rel_vel_std = 0.1653
# tire_acc_std = 36.2566
# tire_vel_std = 0.4602
# com_acc_std = 0.4057
# com_vel_std = 0.01390
# phi_vel_std = 0.01466
# phi_acc_std = 0.01193
# theta_vel_std = 0.01063
# theta_acc_std = 0.00182

body_vel_std = 0.0459
body_acc_std = 0.4031
rel_vel_std = 0.1653
tire_acc_std = 10.2185
tire_vel_std = 0.1594
com_acc_std = 0.1648
com_vel_std = 0.0277
phi_vel_std = 0.0120
phi_acc_std = 0.3730
theta_vel_std = 0.0017
theta_acc_std = 0.1438
theta_pos_std = 0.0124

com_pos_std = 0.0097
tire_pos_std = 0.0124

#obs_cand = [raw_obs[0], raw_obs[6], raw_obs[1], raw_obs[7], (raw_obs[1] - raw_obs[13]), (raw_obs[7] - raw_obs[19]), raw_obs[24], raw_obs[25], raw_obs[30], raw_obs[31]
#         = ... raw_obs[26], raw_obs[32], raw_obs[13], raw_obs[14], raw_obs[19], raw_obs[20]]
#obs_cand = [FLacc, RLacc, FLvel, RLvel, F_relvel, R_relvel, ddz, dz, ddp, dp, // z, p, TFL_vel, TFL_pos, TFR_vel, TFR_pos    ]

with open('./misc/full_car_std_states.csv', newline='') as csvfile:
    sr = csv.reader(csvfile, delimiter=',')
    for ri, row in enumerate(sr):
        if ri == 1:
            obs_std = row

def normalizeObsHalf(env, r=None):
    _buffer = 1.0

    # _r = [0.] * env.state_num
    # for i, std in enumerate(obs_std):
    #     if i < env.state_num:
    #         _r[i] = r[i] / (1 * float(std) * _buffer) # (max * std * buffer)
    _r = [0.] * 16
    _r[0] = r[0] / (1 * body_acc_std * _buffer)
    _r[1] = r[1] / (1 * body_acc_std * _buffer)

    _r[2] = r[2] / (1 * body_vel_std * _buffer)
    _r[3] = r[3] / (1 * body_vel_std * _buffer)

    _r[4] = r[4] / (1 * rel_vel_std * _buffer)
    _r[5] = r[5] / (1 * rel_vel_std * _buffer)

    _r[6] = r[6] / (1 * com_acc_std * _buffer)
    _r[7] = r[7] / (1 * com_vel_std * _buffer)

    _r[8] = r[8] / (1 * theta_acc_std * _buffer)
    _r[9] = r[9] / (1 * theta_vel_std * _buffer)

    _r[10] = r[10] / (1 * com_pos_std * _buffer)
    _r[11] = r[11] / (1 * theta_pos_std * _buffer)
    _r[12] = r[12] / (1 * tire_vel_std * _buffer)
    _r[13] = r[13] / (1 * tire_vel_std * _buffer)
    _r[14] = r[14] / (1 * tire_pos_std * _buffer)
    _r[15] = r[15] / (1 * tire_pos_std * _buffer)


    return _r

# def normalizeObsHalfReward(env, r=None):
#     _buffer = 1.3
#     _r = [0.] * 4
#     _r[0] = r[0] / (1 * com_acc_std * _buffer)
#     _r[1] = r[1] / (1 * com_vel_std * _buffer)
#     _r[2] = r[2] / (1 * theta_acc_std * _buffer)
#     _r[3] = r[3] / (1 * theta_vel_std * _buffer)

#     return _r

def deNormalizeObsHalf(env, r=None):
    _buffer = 1.0

    _r = [0.] * env.state_num_wo_action
    for i, std in enumerate(obs_std):
        if i < env.state_num_wo_action:
            _r[i] = r[i] * (1 * float(std) * _buffer) # (max * std * buffer)

    return _r

def normalizeActionHalf(env, a=None):
    # _a = [0.] * env.action_size
    _a = [0.] * 4
    for i in range(len(_a)):
        _a[i] = a[i] / 70
    
    return _a

def normalizeRoadHalf(env, w=None):
        return w/(0.1 * 0.0769 * 1.1)