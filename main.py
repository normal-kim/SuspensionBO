import sys

# from baselines.common.cmd_util import common_arg_parser, parse_unknown_args

from stable_baselines.common.policies import MlpLstmPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.bench import Monitor

from stable_baselines import SAC, PPO2

from envs.full_car_env import FullCar
from envs.half_car_env import HalfCar
from utils.callbacks import getBestRewardCallback, logDir, rmsLogging
from utils.arg_parser import common_arg_parser, parseLayersFromArgs

import os
import tensorflow as tf

def main(args):
    arg_parser = common_arg_parser()
    args, unknown_args = arg_parser.parse_known_args(args)
    if args.env == 'full_car':
        env = FullCar(args=args)
    elif args.env == 'half_car':
        env = HalfCar(args=args)
    # # env = DummyVecEnv([lambda: env])
    env = Monitor(env, logDir()+args.prefix+"/log", allow_early_resets=True)

    layers = parseLayersFromArgs(args=args) # default [32, 32]
    bestRewardCallback = getBestRewardCallback(args)

    policy_kwargs = dict(layers=layers)

    if args.lstm_on == True:
        policy = MlpLstmPolicy
        policy_kwargs = dict(layers=layers)

    if args.alg == "ppo2":
        if args.lstm_on != True:
            from stable_baselines.common.policies import MlpPolicy
            policy = MlpPolicy
        if args.model_path != None:
            model = PPO2.load(args.model_path, env=env, verbose=1, nminibatches=1, n_steps=1024, tensorboard_log=os.path.join("tensorboard_"+args.env,args.prefix), policy_kwargs=policy_kwargs)
        else:
            model = PPO2(policy, env, verbose=1, nminibatches=1, n_steps=1024, tensorboard_log=os.path.join("tensorboard_"+args.env,args.prefix), policy_kwargs=policy_kwargs)
    elif args.alg == "sac":
        from stable_baselines.sac.policies import MlpPolicy
        env = DummyVecEnv([lambda: env])
        # if args.model_path != None:
        #     # model = SAC.load(args.model_path, env=env, verbose=1, tensorboard_log=os.path.join("tensorboard_"+args.env,args.prefix), policy_kwargs=policy_kwargs)
        #     model = SAC.load(args.model_path, env=env, policy_kwargs=policy_kwargs)
        # else:
            # model = SAC(policy, env, verbose=1, tensorboard_log=os.path.join("tensorboard_"+args.env,args.prefix), policy_kwargs=policy_kwargs)
        model = SAC(MlpPolicy, env, verbose=1, tensorboard_log=os.path.join("tensorboard_"+args.env,args.prefix), policy_kwargs=policy_kwargs)

    

    model.learn(total_timesteps=10*1000000, log_interval=100, callback=bestRewardCallback)
    # model.learn(total_timesteps=2000*1000000, log_interval=100)

if __name__ == '__main__':
    main(sys.argv)

