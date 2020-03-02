from gym import error, spaces
from gym import Env

import matlab.engine
from matlab import double as double_m

import math
import numpy as np
import random

import csv
import os
import copy
from ctypes import *
from sys import platform as _platform

from scipy.signal import welch

from utils.normalize import *
from utils.skyhook import skyhook
from utils.lqr import computeLQR

class HalfCar(Env):
    '''
    Initial values are declared in each function.
    Override the value when calling the function if needed.

    zeta = 0.707
    w0 = 1.
    ts = 0.02
    duration = 16 # in seconds
    '''
    # action augmented : state_num = 35
    def __init__(self, state_num= 4, steps_per_episode=2000, args=None, disable_matlab=False): 
        self.args = args
        print('fullcar_like')
        self.state_num_wo_action = 31
        self.state_num = state_num

        if self.args.lstm_on == True:
            self.observation_space = spaces.Box(low=-1, high=1, shape=(self.args.window_size,state_num,))
        else:
            self.observation_space = spaces.Box(low=-1, high=1, shape=(state_num*self.args.window_size,))

        self.action_size = self.args.action_size
        if self.args.action_cont == "discrete":
            # Discrete Action Space
            self.action_space = spaces.Discrete(2**self.action_size)
        else:
            # Continuous Action Space
            self.action_space = spaces.Box(low=-1., high=1., shape=(2,)) # action_size = 2
            
            self.prefix = args.prefix

        self.episode_cnt = 0
        self.step_cnt = 0

        self.Cmax = 4000
        self.Cmin = 300
        self.R = 0.01
        self.state_X = np.zeros((14,1))

        self.state_SH = [0,0,0,0,0,0,0,0]

        self.steps_per_episode = int(steps_per_episode / (100 / self.args.sampling_freq))

        self.obs_std = []
        with open('./misc/full_car_std_states.csv', newline='') as csvfile:
            sr = csv.reader(csvfile, delimiter=',')
            for ri, row in enumerate(sr):
                if ri == 1:
                    self.obs_std = row

        if disable_matlab == False:
            self.matlab_eng = None
            self._init_matlab()

            self.memory_l = [[0.0]*self.state_num] * self.args.window_size
            self.memory_r = [[0.0]*self.state_num] * self.args.window_size
            self.memory_action = [[0.0]*self.action_size] * self.args.window_size
            self.memory_buffer = [[0.0]*self.state_num_wo_action] * self.args.window_size

        self.action_buffer = [0.0] * self.action_size
    
        self.loadDLL()
        self.done = False
        self.info = {}

    def reset(self):
        self.dll.Fullcar_model_terminate()
        self.dll.Fullcar_model_initialize()

        self.temp_acc = []
        self.temp_phi = []
        self.temp_theta = []

        self.memory_l = [[0.0]*self.state_num] * self.args.window_size
        self.memory_r = [[0.0]*self.state_num] * self.args.window_size

        self.done = False

        self._reset_matlab(self.matlab_eng)

        obs_l = np.asarray([0.0] * self.args.window_size * self.state_num)
        obs_r = np.asarray([0.0] * self.args.window_size * self.state_num)

        return obs_l, obs_r

    def step(self, action_l, action_r):
        info = {}

        raw_obs = self._compute(action_l, action_r, self.matlab_eng)
        
        obs_cand = normalizeObs(self, raw_obs) #obs_candidates
        obs_l = [obs_cand[1], obs_cand[7], (obs_cand[1]-obs_cand[13]), (obs_cand[7]-obs_cand[19])]
        obs_r = [obs_cand[4], obs_cand[10], (obs_cand[4]-obs_cand[16]), (obs_cand[10]-obs_cand[22])]

        self.memorizeBuffer(obs_cand)
        self.memorize_l(obs_l)
        self.memorize_r(obs_r)
        self.memorizeAction(normalizeAction(self, [self.u_fl, self.u_fr, self.u_rl, self.u_rr]))

        reward = 0.
        for i in range(self.args.window_size):
            # reward += max(self.getReward(self.memory[i],self.memory_action[i]),-500000)
            reward += self.getReward(self.memory_buffer[i],self.memory_action[i])
            # reward = self.getReward(obs)

        if self.args.play == True:
            self.csv_logger(self.episode_cnt, self.step_cnt, reward, raw_obs, [self.u_fl, self.u_fr, self.u_rl, self.u_rr], [self.vel_L, self.vel_R], [self.road_FL, self.road_FR])

        done = False
        
        info['saturation_ratio'] = 0

        self.step_cnt += 1

        if self.step_cnt >= self.steps_per_episode:
            done = True
            self.step_cnt = 0
            self.episode_cnt += 1

            # rms = (np.sqrt(np.mean(np.square(np.asarray(self.temp_acc1)))) + np.sqrt(np.mean(np.square(np.asarray(self.temp_acc2)))) + np.sqrt(np.mean(np.square(np.asarray(self.temp_acc3))))
            #    + np.sqrt(np.mean(np.square(np.asarray(self.temp_acc4)))) + np.sqrt(np.mean(np.square(np.asarray(self.temp_acc5)))))/5
            rms = (np.sqrt(np.mean(np.square(np.asarray(self.temp_acc)))) + np.sqrt(np.mean(np.square(np.asarray(self.temp_phi)))) + np.sqrt(np.mean(np.square(np.asarray(self.temp_theta)))))/3
            info['rms'] = rms

            # + np.sqrt(np.mean(np.square(np.asarray(self.temp_acc4)))) + np.sqrt(np.mean(np.square(np.asarray(self.temp_acc5)))))/5
            rms = np.sqrt(np.mean(np.square(np.asarray(self.temp_acc))))
            # print(self.temp_acc, len(self.temp_acc))
            info['rms_com'] = rms

            rms = np.sqrt(np.mean(np.square(np.asarray(self.temp_phi))))
            info['rms_phi'] = rms

            rms = np.sqrt(np.mean(np.square(np.asarray(self.temp_theta))))
            info['rms_theta'] = rms

            self.info = info
            self.done = done
        
        obs_left = np.asarray([item for sub in self.memory_l for item in sub])
        obs_right = np.asarray([item for sub in self.memory_r for item in sub])
        return obs_left, obs_right, done, info


    def _compute(self, action_l, action_r, eng):

        def convertAction(action_l, action_r):
            action_scale_l = (action_l + 1)/2 # convert [-1,1] to [0,1]
            action_scale_r = (action_r + 1)/2
            act0 = convertActionInnerFunc((self.state_SH[0] - self.state_SH[1]),action_scale_l[0])
            act1 = convertActionInnerFunc((self.state_SH[2] - self.state_SH[3]),action_scale_r[0])
            act2 = convertActionInnerFunc((self.state_SH[4] - self.state_SH[5]),action_scale_l[1])
            act3 = convertActionInnerFunc((self.state_SH[6] - self.state_SH[7]),action_scale_r[1])
            return [act0, act1, act2, act3]
        # print(convertAction(action))
        
        def convertActionInnerFunc(vel,scale):
            nominal_damping = 300
            if vel >= 0:
                a = nominal_damping * vel + 1000 * scale
            else:
                a = nominal_damping * vel - 1000 * scale
            return a
            
        action = convertAction(action_l, action_r)

        self.u_fl = action[0]
        self.u_fr = action[1]
        self.u_rl = action[2]
        self.u_rr = action[3]

        i = 0
        while True:
            if i % 5 == 0:
                road_step_cnt = int((100 / self.args.sampling_freq)) * self.step_cnt + int(i / 5)
                self.road_FL = self.road_zl[road_step_cnt][0];self.road_FR = self.road_zr[road_step_cnt][0]
                self.vel_L = self.car_vel[road_step_cnt][0];self.vel_R = self.car_vel[road_step_cnt][0]
                
                self.dll.setRoad_FL(c_float(self.road_FL));self.dll.setRoad_FR(c_float(self.road_FR))
                self.dll.setVel_L(c_float(self.vel_L));self.dll.setVel_R(c_float(self.vel_R))

                self.dll.setU_FL(c_float(self.u_fl));self.dll.setU_FR(c_float(self.u_fr));self.dll.setU_RL(c_float(self.u_rl));self.dll.setU_RR(c_float(self.u_rr))

            self.dll.Fullcar_model_step()

            if i % 5 == 0:
                fl1 = self.dll.getStateFL1();fl2 = self.dll.getStateFL2();fl3 = self.dll.getStateFL3()
                rl1 = self.dll.getStateRL1();rl2 = self.dll.getStateRL2();rl3 = self.dll.getStateRL3()
                fr1 = self.dll.getStateFR1();fr2 = self.dll.getStateFR2();fr3 = self.dll.getStateFR3()
                rr1 = self.dll.getStateRR1();rr2 = self.dll.getStateRR2();rr3 = self.dll.getStateRR3()

                tfl1 = self.dll.getStateTFL1();tfl2 = self.dll.getStateTFL2();tfl3 = self.dll.getStateTFL3()
                trl1 = self.dll.getStateTRL1();trl2 = self.dll.getStateTRL2();trl3 = self.dll.getStateTRL3()
                tfr1 = self.dll.getStateTFR1();tfr2 = self.dll.getStateTFR2();tfr3 = self.dll.getStateTFR3()
                trr1 = self.dll.getStateTRR1();trr2 = self.dll.getStateTRR2();trr3 = self.dll.getStateTRR3()

                z1 = self.dll.getStateZ1();z2 = self.dll.getStateZ2();z3 = self.dll.getStateZ3()
                dphi = self.dll.getStatePhi1();phi = self.dll.getStatePhi2()
                dtheta = self.dll.getStateTheta1();theta = self.dll.getStateTheta2()

                raw_obs = [fl1, fl2, fl3, fr1, fr2, fr3, rl1, rl2, rl3, rr1, rr2, rr3, tfl1, tfl2, tfl3, tfr1, tfr2, tfr3, trl1, trl2, trl3, trr1, trr2, trr3, z1, z2, z3, dphi, phi, dtheta, theta]

                self.temp_acc1 = raw_obs[0]
                self.temp_acc2 = raw_obs[3]
                self.temp_acc3 = raw_obs[6]
                self.temp_acc4 = raw_obs[9]
                self.temp_acc5 = raw_obs[24]

                self.temp_acc.append(raw_obs[24])
                self.temp_phi.append(raw_obs[27])
                self.temp_theta.append(raw_obs[29])

            if i == 5 * (100 / self.args.sampling_freq) - 1:
                fl1 = self.dll.getStateFL1();fl2 = self.dll.getStateFL2();fl3 = self.dll.getStateFL3()
                rl1 = self.dll.getStateRL1();rl2 = self.dll.getStateRL2();rl3 = self.dll.getStateRL3()
                fr1 = self.dll.getStateFR1();fr2 = self.dll.getStateFR2();fr3 = self.dll.getStateFR3()
                rr1 = self.dll.getStateRR1();rr2 = self.dll.getStateRR2();rr3 = self.dll.getStateRR3()

                tfl1 = self.dll.getStateTFL1();tfl2 = self.dll.getStateTFL2();tfl3 = self.dll.getStateTFL3()
                trl1 = self.dll.getStateTRL1();trl2 = self.dll.getStateTRL2();trl3 = self.dll.getStateTRL3()
                tfr1 = self.dll.getStateTFR1();tfr2 = self.dll.getStateTFR2();tfr3 = self.dll.getStateTFR3()
                trr1 = self.dll.getStateTRR1();trr2 = self.dll.getStateTRR2();trr3 = self.dll.getStateTRR3()

                z1 = self.dll.getStateZ1();z2 = self.dll.getStateZ2();z3 = self.dll.getStateZ3()
                dphi = self.dll.getStatePhi1();phi = self.dll.getStatePhi2()
                dtheta = self.dll.getStateTheta1();theta = self.dll.getStateTheta2()

                self.state_X = np.asarray([z3, z2, phi, dphi, theta, dtheta, tfl3, tfl2, tfr3, tfr2, trl3, trl2, trr3, trr2])
                self.state_SH = [fl2,tfl2,fr2,tfr2,rl2,trl2,rr2,trr2]
                state = [fl1, fl2, fl3-0.6, fr1, fr2, fr3-0.6, rl1, rl2, rl3-0.6, rr1, rr2, rr3-0.6, tfl1, tfl2, tfl3-0.3, tfr1, tfr2, tfr3-0.3, trl1, trl2, trl3-0.3, trr1, trr2, trr3-0.3, z1, z2, z3-0.4, dphi, phi, dtheta, theta]
                
                break

            i += 1

        return state

    def getReward(self, obs, action_):
        if self.args.reward == 'vel':
            return -((obs[1] - 0)**2)
        elif self.args.reward == 'acc':
            return -((obs[0] - 0)**2)
        elif self.args.reward == 'acc_vel':
            return -((obs[0] - 0)**2 + (obs[1] - 0)**2 + obs[3]**2 + obs[4]**2 + obs[6]**2 + obs[7]**2 + obs[9]**2 + obs[10]**2 + obs[24]**2 + obs[25]**2)
        elif self.args.reward == 'acc_vel_action':
            return -((obs[0] - 0)**2 + (obs[1] - 0)**2 + obs[3]**2 + obs[4]**2 + obs[6]**2 + obs[7]**2 + obs[9]**2 + obs[10]**2 + obs[24]**2 + obs[25]**2
            + self.R * (action_[0]**2 + action_[1]**2 + action_[2]**2 + action_[3]**2))
        elif self.args.reward == 'com_roll_pitch':
            return -(2*obs[24]**2 + obs[27]**2 + obs[29]**2)
        elif self.args.reward == 'crp':
            return -(2*obs[24]**2 + obs[27]**2 + obs[29]**2)
        elif self.args.reward == 'com_roll_pitch_action':                
            return -(obs[24]**2 + obs[27]**2 + obs[29]**2 + self.R * (action_[0]**2 + action_[1]**2 + action_[2]**2 + action_[3]**2))
        elif self.args.reward == '4_quarters':
            return -((obs[0] - 0)**2 + (obs[1] - 0)**2 + obs[3]**2 + obs[4]**2 + obs[6]**2 + obs[7]**2 + obs[9]**2 + obs[10]**2)
        elif self.args.reward == '4_quarters_acc':
            return -((obs[0] - 0)**2 + obs[3]**2 + obs[6]**2 + obs[9]**2)
        elif self.args.reward == 'half_acc_vel':
            return - (obs[24]**2 + obs[25]**2)
        elif self.args.reward == 'modal':
            x1 = self.T[7][:] @ self.state_X
            x2 = self.T[8][:] @ self.state_X
            x3 = self.T[9][:] @ self.state_X
            x4 = self.T[10][:] @ self.state_X
            x5 = self.T[11][:] @ self.state_X
            x6 = self.T[12][:] @ self.state_X
            return -(x1**2 + x2**2 + x3**2 + x4**2 + x5**2 + x6**2)
        elif self.args.reward == 'psd':
            if len(self.temp_acc) >= 100:
                _acc = self.temp_acc[-100:]
                freq, psd = welch(_acc, 100)
                # print(freq)
                # print(psd)
                # exit(0)
                return -np.mean(psd)
            else:
                return -100

        elif self.args.reward == 'psd_max':
            if len(self.temp_acc) >= 100:
                _acc = self.temp_acc[-100:]
                freq, psd = welch(_acc, 100)
                # print(freq)
                # print(psd)
                # exit(0)
                return -np.max(psd)
            else:
                return -100
        elif self.args.reward == 'psd_max_acc_vel':
            if len(self.temp_acc) >= 100:
                _acc = self.temp_acc[-100:]
                freq, psd = welch(_acc, 100)
                # print(freq)
                # print(psd)
                # exit(0)
                # return -np.max(psd)
                return -((obs[0] - 0)**2 + (obs[1] - 0)**2 + np.max(psd))
            else:
                return -100
        else:
            return -(2*obs[24]**2 + obs[27]**2 + obs[29]**2)

    def skyhook(self):
        return skyhook(self)

    def memorize_l(self, obs):
        _temp = copy.deepcopy(self.memory_l)
        for i in range(self.args.window_size):
            if i < self.args.window_size-1:
                self.memory_l[i+1] = _temp[i]

        self.memory_l[0] = obs

    def memorize_r(self, obs):
        _temp = copy.deepcopy(self.memory_r)
        for i in range(self.args.window_size):
            if i < self.args.window_size-1:
                self.memory_r[i+1] = _temp[i]

        self.memory_r[0] = obs

    def memorizeBuffer(self, obs):
        _temp = copy.deepcopy(self.memory_buffer)
        for i in range(self.args.window_size):
            if i < self.args.window_size-1:
                self.memory_buffer[i+1] = _temp[i]

        self.memory_buffer[0] = obs

    def memorizeAction(self,action):
        _temp = copy.deepcopy(self.memory_action)
        for i in range(self.args.window_size):
            if i < self.args.window_size-1:
                self.memory_action[i+1] = _temp[i]
        
        self.memory_action[0] = action

    def actionBuffer(self, action):
        if self.args.delay_size >= 1:
            _a = self.action_buffer[0]
            self.action_buffer[0] = action
            return _a
        else:
            return action

    def loadDLL(self):
        CDLL._func_restype_ = c_float
        if _platform == "linux" or _platform == "linux2":
            # linux
            print("No dll for linux")
            exit(0)
        elif _platform == "darwin":
            # MAC OS X
            self.dll = CDLL("misc/Fullcar_model_osx.dylib")
            print("MAC dll")
        else:
            self.dll = CDLL("misc/Fullcar_model_win64_ver2.dll")
            print("Windows dll")
        
        self.dll.Fullcar_model_initialize()
        self.dll.restype = c_float

    def _init_matlab(self):
        if self.matlab_eng == None:
            # Launch Matlab if not running
            self.matlab_eng = matlab.engine.start_matlab()
            self.matlab_eng.desktop(nargout=0)
            self.matlab_eng.addpath(r'matlab')  
            print("Matlab Loaded")

        self._reset_matlab(self.matlab_eng)

        self.matlab_eng.SSgenerator()
        self.A = self.matlab_eng.eval('A')
        self.B = self.matlab_eng.eval('B')

    def _terminate(self):
        self.matlab_eng.quit()

    def _reset_matlab(self, eng):
        self.matlab_cnt = 0.0
        # self.m1_acc = 0; self.m1_vel = 0; self.m1_pos = 0; self.m2_acc = 0; self.m2_vel = 0; self.m2_pos = 0


        #-----------------------#
        # Road Road_Generator_Quarter(index)
        # index
        # 1 : ISO 8608
        # 2 : random sine
        # 3 : sine with amplitude of ISO 8608 in frequency domain 
        #-----------------------#
        # Road_Generator_Quarter_tester(index, roughness)
        # index is same
        # roughness 
        # 1 : rough road
        # 2 : soft road
        # 3 : bumper roads
        #-----------------------#
        if self.args.road_type == "iso8608":
            eng.Road_Generator_Fullcar(1.0,0.0)
        elif self.args.road_type == "random_sine":
            eng.Road_Generator_Fullcar(2.0,0.0)
        elif self.args.road_type == "pinpoint_sine":
            eng.Road_Generator_Fullcar(3.0,0.0)
        elif self.args.road_type == "only_soft":
            eng.Road_Generator_Fullcar(1.0,2.0)
        elif self.args.road_type == "only_rough":
            eng.Road_Generator_Fullcar(1.0,1.0)
        elif self.args.road_type == "only_bumper":
            eng.Road_Generator_Fullcar(1.0,3.0)
        # elif self.args.road_type == "no_bumper":
        #     eng.Road_Generator_Quarter_tester(1.0,4.0)
        else:
            eng.Road_Generator_Fullcar(1.0,0.0)

        # eng.matrixGenerator()

        # sa = eng.eval('sys_A')
        # sb = eng.eval('sys_B')
        # self.sys_A = np.array(sa)
        # self.sys_B = np.array(sb)
        # self.road_time = eng.eval('t')
        # self.road_z = eng.eval('Road_z')
        # self.road_index = eng.eval('Road_index')
        # self.kx = eng.eval('Kx')

        self.road_time = eng.eval('t')
        self.road_zl = eng.eval('road_ZL')
        self.road_zr = eng.eval('road_ZR')
        self.car_vel = eng.eval('car_vel')
        self.road_index = eng.eval('Road_index')
        self.Kx = eng.eval('Kx')
        self.T = np.asarray(eng.eval('transMat')) # it is already inversed matrix! i.e. inv(M)

    def render(self, mode):
        pass

    def csv_logger(self, e, s, r, obs, a, cv, ri):
        row = [e, s, r] + obs + a + cv + ri
        if os.path.isfile('./logs/'+self.args.prefix+'.csv'):
            with open('./logs/'+self.args.prefix+'.csv','a') as fd:
                writer = csv.writer(fd)
                writer.writerow(row)
        else:
            with open('./logs/'+self.args.prefix+'.csv','w') as fd:
                writer = csv.writer(fd)
                writer.writerow(['episode', 'step', 'reward', 'FL_acc','FL_vel','FL_pos','FR_acc','FR_vel','FR_pos','RL_acc','RL_vel','RL_pos','RR_acc','RR_vel','RR_pos','TFL_acc','TFL_vel','TFL_pos','TFR_acc','TFR_vel','TFR_pos','TRL_acc','TRL_vel','TRL_pos','TRR_acc','TRR_vel','TRR_pos','body_acc','body_vel','body_pos','phi_vel','phi_pos','theta_vel','theta_pos','u_FL','u_FR','u_RL','u_RR','car_vel_l','car_vel_r','road_input_l','road_input_r'])
                writer.writerow(row)
