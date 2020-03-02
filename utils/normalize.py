import csv

obs_std = []
with open('./misc/full_car_std_states_2.csv', newline='') as csvfile:
    sr = csv.reader(csvfile, delimiter=',')
    for ri, row in enumerate(sr):
        if ri == 1:
            obs_std = row

def normalizeObs(env, r=None):
    _buffer = 1.0

    _r = [0.] * env.state_num_wo_action
    for i, std in enumerate(obs_std):
        if i < env.state_num_wo_action:
            _r[i] = r[i] / (1 * float(std) * _buffer) # (max * std * buffer)

    return _r

def deNormalizeObs(env, r=None):
    _buffer = 1.0

    _r = [0.] * env.state_num_wo_action
    for i, std in enumerate(obs_std):
        if i < env.state_num_wo_action:
            _r[i] = r[i] * (1 * float(std) * _buffer) # (max * std * buffer)

    return _r

def normalizeAction(env, a=None):
    # _a = [0.] * env.action_size
    _a = [0.] * 4
    for i in range(len(_a)):
        _a[i] = a[i] / (1 * float(obs_std[33+i]) * 1.0)
    
    return _a

def normalizeRoad(env, w=None):
        return w/(0.1 * 0.0769 * 1.1)
