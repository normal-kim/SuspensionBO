import sys

from stable_baselines.common.policies import MlpLstmPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.bench import Monitor

from stable_baselines import SAC, PPO2

from envs.full_car_env import FullCar
from envs.half_car_env import HalfCar
from utils.callbacks import getBestRewardCallback, logDir, rmsLogging
from utils.arg_parser import common_arg_parser, parseLayersFromArgs

import os
from glob import glob
import tensorflow as tf

import csv
import re

import json

from PyInquirer import prompt, print_json
from argparse import Namespace

def main(args):
    if args.env == 'full_car':
        env = FullCar(args=args)
    elif args.env == 'half_car':
        env = HalfCar(args=args)

    layers = parseLayersFromArgs(args=args) # default [32, 32]
    policy_kwargs = dict(layers=layers)
    env = DummyVecEnv([lambda: env])
    # if args.lstm_on == True:
    #     policy = MlpLstmPolicy
    #     policy_kwargs = dict(layers=layers)

    if args.alg == "ppo2":
        # model = PPO2(policy, env, verbose=1, nminibatches=1, n_steps=64, policy_kwargs=policy_kwargs)
        model = PPO2.load(args.model_path, env=env)
    elif args.alg == "sac":
        # from stable_baselines.sac.policies import MlpPolicy
        # # model = SAC(policy, env, verbose=1, tensorboard_log=os.path.join("tensorboard_"+args.env,args.prefix), policy_kwargs=policy_kwargs)
        # model = SAC(MlpPolicy, env, verbose=1, policy_kwargs=policy_kwargs)
        model = SAC.load(args.model_path, env=env)
    
    # model.load(sorted(os.listdir(logDir()+args.prefix+"*"))[-1])
    # model = model.load(args.model_path, env=env)

    obs = env.reset()

    test_runs = 50
    rms_com = 0.
    rms_phi = 0.
    rms_theta = 0.
    for i in range(test_runs):
        print(i+1,"/",test_runs)
        while True:
            action, _states = model.predict(obs)
            obs, rewards, dones, info = env.step(action)

            if dones:
                rms_com += info[0]['rms_com']
                rms_phi += info[0]['rms_phi']
                rms_theta += info[0]['rms_theta']

                break

    print("COM z RMS :", rms_com/test_runs)
    print("phi RMS :", rms_phi/test_runs)
    print("theta RMS :", rms_theta/test_runs)

if __name__ == '__main__':
    arg_parser = common_arg_parser()
    args, unknown_args = arg_parser.parse_known_args(sys.argv)

    log_files = glob(logDir()+"/*")

    questions = [
        {
            'type': 'list',
            'name': 'target_model',
            'message': 'Which run run run?',
            'choices':log_files
        }
    ]

    answers = prompt(questions)

    f = open(answers['target_model']+'/log.monitor.csv', 'r')
    _args = json.loads(f.readline().replace('#',''))['args']
    _args['play'] = True

    model_files = sorted(glob(answers['target_model'].replace('.monitor.csv','')+'/*_model.pkl'))
    model_files.sort(key=lambda var:[int(x) if x.isdigit() else x for x in re.findall(r'[^0-9]|[0-9]+', var)])

    _args['model_path'] = model_files[-1]

    if args.state_reconstruction != "None":
        _args['state_reconstruction'] = args.state_reconstruction
    
    if args.prefix != "":
        _args['prefix'] = args.prefix

    args = Namespace(**_args)
    print("Load saved args", args)
    f.close()

    main(args=args)

