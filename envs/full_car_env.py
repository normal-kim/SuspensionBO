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

import tensorflow as tf

class FullCar(Env):
    '''
    Initial values are declared in each function.
    Override the value when calling the function if needed.

    zeta = 0.707
    w0 = 1.
    ts = 0.02
    duration = 16 # in seconds
    '''
    # action augmented : state_num = 35
    def __init__(self, state_num=8, steps_per_episode=2000, args=None, disable_matlab=False): 
        self.args = args

        self.state_num_wo_action = 33

        if self.args.road_cls != "none":
            # Road Classification 넣기
            state_num += 1

            if self.args.road_cls == "ground_truth":
                pass
            elif self.args.road_cls == "neural_network":
                print("아직임")
                exit(0)

        self.state_num = state_num

        if self.args.state_reconstruction != 'None':
            if self.args.state_reconstruction == 'nn':
                print("State Reconstruction : NN")

                from keras.models import load_model

                self.memory_sr_acc = [[0.0]*9] * self.args.state_recon_window_size
                self.memory_sr_roll = [[0.0]] * self.args.state_recon_window_size
                self.memory_sr_pitch = [[0.0]] * self.args.state_recon_window_size
                self.memory_sr_car_vel = [0.0] * self.args.state_recon_window_size

                self.sr_model = load_model('misc/state_reconstruction.hdf5', custom_objects={"tf": tf})
                x = [np.zeros((1,self.args.state_recon_window_size,9)),np.zeros((1,self.args.state_recon_window_size,1)),np.zeros((1,self.args.state_recon_window_size,1)),np.zeros((1,self.args.state_recon_window_size))]
                    
                self.sr_model.predict(x) # Firing up
                
            elif self.args.state_reconstruction == 'kf':
                self.resetKKFVar()
                print("State Reconstruction : Kalman")


        if self.args.lstm_on == True:
            self.observation_space = spaces.Box(low=-1, high=1, shape=(self.args.window_size,state_num,))
        else:
            self.observation_space = spaces.Box(low=-1, high=1, shape=(state_num*self.args.window_size,))

        self.action_size = self.args.action_size
        if self.args.decoupling :
            self.action_size = 3

        if self.args.action_cont == "discrete":
            # Discrete Action Space
            self.action_space = spaces.Discrete(2**self.action_size)
        else:
            # Continuous Action Space
            self.action_space = spaces.Box(low=-1., high=1., shape=(self.action_size,))
            
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

            self.memory = [[0.0]*self.state_num] * self.args.window_size
            self.memory_action = [[0.0]*self.action_size] * self.args.window_size
            self.memory_buffer = [[0.0]*self.state_num_wo_action] * self.args.window_size

            # self.summary_writer = tf.summary.FileWriter("./tensorboard_" + args.env + "/" + args.prefix)

        self.action_buffer = [0.0] * self.action_size
    
        self.loadDLL()
        self.done = False
        self.info = {}

        # self.normalizeObs    = normalizeObs(self)
        # self.deNormalizeObs  = deNormalizeObs(self)
        # self.normalizeAction = normalizeAction(self)

    def reset(self):
        self.dll.Fullcar_model_terminate()
        self.dll.Fullcar_model_initialize()

        self.temp_acc = []
        self.temp_phi = []
        self.temp_theta = []

        self.done = False

        self.memory = [[0.0]*self.state_num] * self.args.window_size
        self.memory_action = [[0.0]*self.action_size] * self.args.window_size
        self.memory_buffer = [[0.0]*self.state_num_wo_action] * self.args.window_size

        if self.args.state_reconstruction != 'None':
            if self.args.state_reconstruction == 'nn':
                self.memory_sr_acc = [[0.0]*9] * self.args.state_recon_window_size
                self.memory_sr_roll = [[0.0]] * self.args.state_recon_window_size
                self.memory_sr_pitch = [[0.0]] * self.args.state_recon_window_size
                self.memory_sr_car_vel = [0.0] * self.args.state_recon_window_size
                
            elif self.args.state_reconstruction == 'kf':
                self.resetKKFVar()

        self._reset_matlab(self.matlab_eng)

        if "psd" in self.args.reward:
            # run 100 steps to collect data for PSD cal.
            for i in range(100):
                r, _, _, _ = self.step([0.])
            
            return r
        else:
            if self.args.lstm_on != True:
                return np.asarray([0.0] * self.args.window_size * self.state_num)
            else:
                return np.asarray([[0.0] * self.state_num] * self.args.window_size)
            

    def skyhookImitation(self, info):
        sh_actions = self.skyhook()
        sh_new_obs = self._compute(action=sh_actions, eng=self.matlab_eng, bypass_action=True)

        sh_reward = self.getReward(sh_new_obs, sh_actions)

        # a1 = action % 2
        # a2 = (action // 2) % 2
        # a3 = ((action // 2) // 2) % 2
        # a4 = (((action // 2) // 2) // 2) % 2
        def convertDampingForceToAction(f, v1, v2):
            return int(((f / (v1 - v2)) - 300) / 3700)

        _a = []
        raw_obs = deNormalizeObs(self, self.memory[0])
        for sh_index, sh_action in enumerate(list(reversed(sh_actions))):
            
            __a = convertDampingForceToAction(sh_action, raw_obs[2+3*sh_index],  raw_obs[14+3*sh_index])
            _a.append(__a)

        sh_action = 0
        for ele in _a: 
            sh_action = (sh_action << 1) | ele 

        info['sh_action'] = sh_action
        info['sh_new_obs'] = sh_new_obs
        info['sh_reward'] = sh_reward

        return info

    def step(self, action):
        '''
        Computes system with given action
        returns y_t and y_t - y_t-1 as states
        reward is calculated with given time_index
        '''
        info = {}
        # SKYHOOK Imitation ; calculates sh control
        if self.args.imitation == True:
            self.skyhookImitation(info)

        raw_obs = self._compute(action, self.matlab_eng)
        
        # obs = normalizeObs(self, raw_obs)

        obs_cand = normalizeObs(self, raw_obs) #obs_candidates
        obs = [obs_cand[1], obs_cand[4], obs_cand[7], obs_cand[10], (obs_cand[1]-obs_cand[13]), (obs_cand[4]-obs_cand[16]), (obs_cand[7]-obs_cand[19]), (obs_cand[10]-obs_cand[22])]
        # obs.extend(normalizeAction(self, [self.u_fl, self.u_fr, self.u_rl, self.u_rr]))
        # obs.append(self.vel_L/40)

        if self.args.state_reconstruction != 'None':
            self.memorizeBuffer(normalizeObs(self, self.state_gt))
        else:
            self.memorizeBuffer(obs_cand)

        self.memorize(obs)

        # obs.extend(normalizeAction(self, [self.u_fl, self.u_fr, self.u_rl, self.u_rr]))
        # obs.append(self.vel_L/40)
        
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
        
        if self.args.lstm_on != True:
            flat_obs = np.asarray([item for sub in self.memory for item in sub])
            return flat_obs, reward, done, info
        else:
            return np.asarray(self.memory), reward, done, info

    def _compute(self, action, eng):
        # tic = time.time()

        def convertAction(action):
            action_scale = (action + 1)/2 # convert [-1,1] to [0,1]
            act0 = convertActionInnerFunc((self.state_SH[0] - self.state_SH[1]),action_scale[0])
            act1 = convertActionInnerFunc((self.state_SH[2] - self.state_SH[3]),action_scale[1])
            act2 = convertActionInnerFunc((self.state_SH[4] - self.state_SH[5]),action_scale[2])
            act3 = convertActionInnerFunc((self.state_SH[6] - self.state_SH[7]),action_scale[3])
            return [act0, act1, act2, act3]
        # print(convertAction(action))
        
        def convertActionInnerFunc(vel,scale):
            nominal_damping = 300
            if vel >= 0:
                a = nominal_damping * vel + 1000 * scale
            else:
                a = nominal_damping * vel - 1000 * scale
            return a

        def transformDecoupling(action):
            a = 2.65/2; b = 2.65/2; w = 1.568/2
            Tmat = np.asarray([[b/(2*(a+b)), -1/(2*(a+b)), 1/(2*w)],[b/(2*(a+b)),-1/(2*(a+b)),-1/(2*w)],[a/(2*(a+b)),1/(2*(a+b)),1/(2*w)],[a/(2*(a+b)), 1/(2*(a+b)),-1/(2*w)]])
            act = Tmat @ action
            return act

        if self.args.decoupling:
            action = transformDecoupling(action)
            action = convertAction(action)
            self.u_fl = action[0]
            self.u_fr = action[1]
            self.u_rl = action[2]
            self.u_rr = action[3]             

        elif self.args.action_type == "damping_force" and self.args.action_cont == "continuous":
            
            action = convertAction(action)

            self.u_fl = action[0]
            self.u_fr = action[1]
            self.u_rl = action[2]
            self.u_rr = action[3]

        elif self.args.action_type == "damping_force" and self.args.action_cont == "discrete":
            a1 = action % 2
            a2 = (action // 2) % 2
            a3 = ((action // 2) // 2) % 2
            a4 = (((action // 2) // 2) // 2) % 2

            self.u_fl = (a1-0.5) * 1000
            self.u_fr = (a2-0.5) * 1000
            self.u_rl = (a3-0.5) * 1000
            self.u_rr = (a4-0.5) * 1000

        elif self.args.action_type == "damping_constant" and self.args.action_cont == "continuous":
            def convertDampingConstantAction(action, v1, v2):
                # Action as Damping Constant                
                return (v1 - v2) * (300 + 3700*(action+1)/2)

            raw_obs = deNormalizeObs(self, r=self.memory[0])
            self.u_fl = convertDampingConstantAction(action[0], raw_obs[1],  raw_obs[13])
            self.u_fr = convertDampingConstantAction(action[1], raw_obs[4],  raw_obs[16])
            self.u_rl = convertDampingConstantAction(action[2], raw_obs[7],  raw_obs[19])
            self.u_rr = convertDampingConstantAction(action[3], raw_obs[10], raw_obs[22])

        elif self.args.action_type == "damping_constant" and self.args.action_cont == "discrete":
            a1 = action % 2
            a2 = (action // 2) % 2
            a3 = ((action // 2) // 2) % 2
            a4 = (((action // 2) // 2) // 2) % 2
        
            def convertDampingConstantAction(action, v1, v2):
                # Action as Damping Constant                
                return (v1 - v2) * (300 + 3700*action)

            raw_obs = deNormalizeObs(self, r=self.memory[0])
            self.u_fl = convertDampingConstantAction(a1, raw_obs[1],  raw_obs[13])
            self.u_fr = convertDampingConstantAction(a2, raw_obs[4],  raw_obs[16])
            self.u_rl = convertDampingConstantAction(a3, raw_obs[7],  raw_obs[19])
            self.u_rr = convertDampingConstantAction(a4, raw_obs[10], raw_obs[22])

        elif self.args.symmetric:
            self.u_fl = action[0] * 500
            self.u_fr = action[2] * 500
            self.u_rl = action[1] * 500
            self.u_rr = action[3] * 500

        elif self.args.action_type == "skyhook":

            def convertActionSkyhook(action):
                [u_fl,u_fr,u_rl,u_rr] = self.skyhook()
                scaler = 100
                buffer = [u_fl + scaler*action[0], u_fr + scaler*action[1], u_rl + scaler*action[2], + u_rr + scaler*action[3]]
                # print(buffer)
                return buffer
                
            action = convertActionSkyhook(action)
            self.u_fl = action[0]
            self.u_fr = action[1]
            self.u_rl = action[2]
            self.u_rr = action[3]
            bypass_action = True

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
                ddphi = self.dll.getStatePhiAcc(); ddtheta = self.dll.getStateThetaAcc()

                raw_obs = [fl1, fl2, fl3-0.6, fr1, fr2, fr3-0.6, rl1, rl2, rl3-0.6, rr1, rr2, rr3-0.6, tfl1, tfl2, tfl3-0.3, tfr1, tfr2, tfr3-0.3, trl1, trl2, trl3-0.3, trr1, trr2, trr3-0.3, z1, z2, z3-0.4, ddphi, dphi, phi, ddtheta, dtheta, theta]

                self.temp_acc1 = raw_obs[0]
                self.temp_acc2 = raw_obs[3]
                self.temp_acc3 = raw_obs[6]
                self.temp_acc4 = raw_obs[9]
                self.temp_acc5 = raw_obs[24]

                self.temp_acc.append(raw_obs[24])
                self.temp_phi.append(raw_obs[27])
                self.temp_theta.append(raw_obs[30])

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
                ddphi = self.dll.getStatePhiAcc(); ddtheta = self.dll.getStateThetaAcc()


                if self.args.noise_on :
                    noise_level = 0.03
                    fl2 += 0.018 * np.random.randn() * noise_level # noise std * N(0,1) * scale
                    tfl2 += 0.45 * np.random.randn() * noise_level
                    fr2 += 0.018 * np.random.randn() * noise_level
                    tfr2 += 0.45 * np.random.randn() * noise_level
                    rl2 += 0.018 * np.random.randn() * noise_level
                    trl2 += 0.45 * np.random.randn() * noise_level
                    rr2 += 0.018 * np.random.randn() * noise_level
                    trr2 += 0.45 * np.random.randn() * noise_level

                self.state_X = np.asarray([z3, z2, phi, dphi, theta, dtheta, tfl3, tfl2, tfr3, tfr2, trl3, trl2, trr3, trr2])
                self.state_SH = [fl2,tfl2,fr2,tfr2,rl2,trl2,rr2,trr2]
                state = [fl1, fl2, fl3-0.6, fr1, fr2, fr3-0.6, rl1, rl2, rl3-0.6, rr1, rr2, rr3-0.6, tfl1, tfl2, tfl3-0.3, tfr1, tfr2, tfr3-0.3, trl1, trl2, trl3-0.3, trr1, trr2, trr3-0.3, z1, z2, z3-0.4, ddphi, dphi, phi, ddtheta, dtheta, theta]
                
                break

            i += 1

        if self.args.state_reconstruction != 'None':
            self.state_gt = state
            if self.args.state_reconstruction == 'nn':
                
                # memorizeSR은 memorize와 같은데 다만 첫번째 변수에 두번째 값을 밀어넣어서 저장함 / 기본 window_size 8
                self.memorizeSR(self.memory_sr_acc, [fl1, fr1, rl1, rr1, tfl1, tfr1, trl1, trr1])
                self.memorizeSR(self.memory_sr_roll, [dphi])
                self.memorizeSR(self.memory_sr_pitch, [dtheta])
                self.memorizeSR(self.memory_sr_car_vel, self.vel_L)

                x = [np.asarray(self.memory_sr_acc),np.asarray(self.memory_sr_roll),np.asarray(self.memory_sr_pitch),np.asarray(self.memory_sr_car_vel)]
                    
                predictions = self.sr_model.predict(x)

                vel_body_f_prediction = predictions[0][0]
                vel_body_b_prediction = predictions[1][0]
                vel_tire_f_prediction = predictions[2][0]
                vel_tire_b_prediction = predictions[3][0]
                pos_body_f_prediction = predictions[4][0]
                pos_body_b_prediction = predictions[5][0]
                pos_tire_f_prediction = predictions[6][0]
                pos_tire_b_prediction = predictions[7][0]
                z_dot_prediction = predictions[8][0]
                z_prediction = predictions[9][0]
                phi_prediction = predictions[10][0]
                theta_prediction = predictions[11][0]

                state = [
                    fl1, vel_body_f_prediction[0] * self.obs_std[1], pos_body_f_prediction[0] * self.obs_std[2],
                    fr1, vel_body_f_prediction[1] * self.obs_std[4], pos_body_f_prediction[1] * self.obs_std[5],
                    rl1, vel_body_b_prediction[0] * self.obs_std[7], pos_body_b_prediction[0] * self.obs_std[8],
                    rr1, vel_body_b_prediction[1] * self.obs_std[10], pos_body_b_prediction[1] * self.obs_std[11],
                    tfl1, vel_tire_f_prediction[0] * self.obs_std[13], pos_tire_f_prediction[0] * self.obs_std[14],
                    tfr1, vel_tire_f_prediction[1] * self.obs_std[16], pos_tire_f_prediction[1] * self.obs_std[17],
                    trl1, vel_tire_b_prediction[0] * self.obs_std[19], pos_tire_b_prediction[0] * self.obs_std[20],
                    trr1, vel_tire_b_prediction[1] * self.obs_std[22], pos_tire_b_prediction[1] * self.obs_std[23],
                    z1, z_dot_prediction * self.obs_std[25], z_prediction * self.obs_std[26],
                    dphi, phi_prediction * self.obs_std[28],
                    dtheta, theta_prediction * self.obs_std[30]
                    ]

            elif self.args.state_reconstruction == 'kf':
                # [[fl_kk3],[fl_kk2]] = self.kkf(fl1,0,'fl')
                # [[rl_kk3],[rl_kk2]] = self.kkf(rl1,0,'rl')
                # [[fr_kk3],[fr_kk2]] = self.kkf(fr1,0,'fr')
                # [[rr_kk3],[rr_kk2]] = self.kkf(rr1,0,'rr')

                # [[tfl_kk3],[tfl_kk2]] = self.kkf(fl1 - tfl1,1,'tfl')
                # [[trl_kk3],[trl_kk2]] = self.kkf(rl1 - trl1,1,'trl')
                # [[tfr_kk3],[tfr_kk2]] = self.kkf(fr1 - tfr1,1,'tfr')
                # [[trr_kk3],[trr_kk2]] = self.kkf(rr1 - trr1,1,'trr')

                [[fl_kk2]] = self.kkf(fl1,0,'fl')
                [[rl_kk2]] = self.kkf(rl1,0,'rl')
                [[fr_kk2]] = self.kkf(fr1,0,'fr')
                [[rr_kk2]] = self.kkf(rr1,0,'rr')

                [[rel_fl_kk2]] = self.kkf(fl1 - tfl1,1,'tfl')
                [[rel_rl_kk2]] = self.kkf(rl1 - trl1,1,'trl')
                [[rel_fr_kk2]] = self.kkf(fr1 - tfr1,1,'tfr')
                [[rel_rr_kk2]] = self.kkf(rr1 - trr1,1,'trr')

                # [[z_kk3],[z_kk2]] = self.kkf(z1,2,'z')
                # [[phi_kk]] = self.kkf(dphi,3,'phi')
                # [[theta_kk]] = self.kkf(dtheta,3,'theta')

                state = [
                    fl1, fl_kk2, fl3,
                    fr1, fr_kk2, fr3,
                    rl1, rl_kk2, rl3,
                    rr1, rr_kk2, rr3,
                    tfl1, fl_kk2 - rel_fl_kk2, tfl3,
                    tfr1, fr_kk2 - rel_fr_kk2, tfr3,
                    trl1, rl_kk2 - rel_rl_kk2, trl3,
                    trr1, rr_kk2 - rel_rr_kk2, trr3,
                    z1, z2, z3,
                    dphi, phi,
                    dtheta, theta
                ]

        return state

    def getReward(self, obs, action_):
        if self.args.reward == 'acc_vel':
            return -((obs[0] - 0)**2 + (obs[1] - 0)**2 + obs[3]**2 + obs[4]**2 + obs[6]**2 + obs[7]**2 + obs[9]**2 + obs[10]**2 + obs[24]**2 + obs[25]**2)
        elif self.args.reward == 'com_roll_pitch' or self.args.reward == 'crp':
            return -(obs[24]**2 + obs[25]**2 + obs[27]**2 + obs[30]**2)
        elif self.args.reward == 'crp2':
            return -(obs[24]**2 + obs[27]**2 + obs[30]**2)
        elif self.args.reward == 'crp3':
            return -(obs[24]**2 + obs[25]**2 + obs[27]**2 + obs[28]**2 + obs[30]**2 + obs[31]**2)
        elif self.args.reward == '4_quarters':
            return -((obs[0] - 0)**2 + (obs[1] - 0)**2 + obs[3]**2 + obs[4]**2 + obs[6]**2 + obs[7]**2 + obs[9]**2 + obs[10]**2)
        elif self.args.reward == '4_quarters_acc':
            return -((obs[0] - 0)**2 + obs[3]**2 + obs[6]**2 + obs[9]**2)
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

    def memorizeSR(self, target, data):
        _temp = copy.deepcopy(target)
        for i in range(self.args.window_size):
            if i < self.args.window_size-1:
                target[i+1] = _temp[i]

        target[0] = data

        return target

    def memorize(self, obs):
        _temp = copy.deepcopy(self.memory)
        for i in range(self.args.window_size):
            if i < self.args.window_size-1:
                self.memory[i+1] = _temp[i]

        self.memory[0] = obs

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
            self.dll = CDLL("misc/Fullcar_model_win64.dll")
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

    def resetKKFVar(self):
        self.P_phi = np.zeros((1))
        self.P_theta = np.zeros((1))
        self.P_z = np.zeros((1))
        self.P_tfl = np.zeros((1))
        self.P_tfr = np.zeros((1))
        self.P_trl = np.zeros((1))
        self.P_trr = np.zeros((1))
        self.P_fl = np.zeros((1))
        self.P_fr = np.zeros((1))
        self.P_rl = np.zeros((1))
        self.P_rr = np.zeros((1))

        self.hx_z = np.zeros((1))
        self.hx_phi = np.zeros((1))
        self.hx_theta = np.zeros((1))
        self.hx_tfl = np.zeros((1))
        self.hx_tfr = np.zeros((1))
        self.hx_trl = np.zeros((1))
        self.hx_trr = np.zeros((1))
        self.hx_fl = np.zeros((1))
        self.hx_fr = np.zeros((1))
        self.hx_rl = np.zeros((1))
        self.hx_rr = np.zeros((1))
        
        self.P_total = np.zeros((14,14))
        self.hx_total = np.zeros((14,1))
        self.u = np.zeros((4,1))

        self.x_fl = np.zeros((2,1))
        self.x_fr = np.zeros((2,1))
        self.x_rl = np.zeros((2,1))
        self.x_rr = np.zeros((2,1))

        self.x_tfl = np.zeros((2,1))
        self.x_tfr = np.zeros((2,1))
        self.x_trl = np.zeros((2,1))
        self.x_trr = np.zeros((2,1))

    def kkf(self,in_,index,position): # kinematic kalman filter
        if index == 0: # body position
            
            w1 = 1.6650
            v1 = -1.1415
            V = 10**v1
            W = 10**w1
            T = 0.01
            C = np.eye(1)
            Phi = 1
            Gamma = T
            Tau = T
            ## 0.5~25Hz filtering matrix

            a = np.array([[0.9843,-0.01571],
                         [1,0]])
            b = np.array([[0.4921],[0]])
            c = np.array([[0.9843, -1.0157]])
            d = np.array([[0.4921]])

            if position == 'fl':
                P = self.P_fl
                hx = self.hx_fl
                acc = c @ self.x_fl + d*in_
            elif position == 'fr':
                P = self.P_fr
                hx = self.hx_fr
                acc = c @ self.x_fr + d*in_
            elif position == 'rl':
                P = self.P_rl
                hx = self.hx_rl
                acc = c @ self.x_rl + d*in_
            elif position == 'rr':
                P = self.P_rr
                hx = self.hx_rr
                acc = c @ self.x_rr + d*in_

            K = P * np.transpose(C) * np.linalg.inv(C * P * np.transpose(C) + V)
            zk = hx + acc * T
            xk = hx + K * (zk - C * hx)
            p_buff = (np.eye(1) - K * C) * P
            hx = Phi * xk + Gamma * acc
            P = Phi * p_buff * np.transpose(Phi) + (Tau * W) * np.transpose(Tau)

            if position == 'fl':
                self.P_fl = P
                self.hx_fl = hx
                self.x_fl = a @ self.x_fl + b * in_
            elif position == 'fr':
                self.P_fr = P
                self.hx_fr = hx
                self.x_fr = a @ self.x_fr + b * in_
            elif position == 'rl':
                self.P_rl = P
                self.hx_rl = hx
                self.x_rl = a @ self.x_rl + b * in_
            elif position == 'rr':
                self.P_rr = P
                self.hx_rr = hx
                self.x_rr = a @ self.x_rr + b * in_

        elif index == 1: # !!!!body - tire value!!!!!, i.e. in_ = FL1 - TFL1 >> output is also FL2 - TFL2 cause that works better
            
            w1 = -0.7034
            v1 = 1.8197
            T = 0.01
            Phi = 1
            Gamma = T
            Tau = T
            C = np.eye(1)
            V = 10**v1
            W = 10**w1
            ## 5~10Hz filtering matrix
            a = np.array([[1.0515,-0.3249],
                         [1,0]])
            b = np.array([[0.3375],[0]])
            c = np.array([[1.0515, -1.3249]])
            d = np.array([[0.3375]])


            if position == 'tfl':
                P = self.P_tfl
                hx = self.hx_tfl
                acc = c @ self.x_tfl + d*in_
            elif position == 'tfr':
                P = self.P_tfr
                hx = self.hx_tfr
                acc = c @ self.x_tfr + d*in_
            elif position == 'trl':
                P = self.P_trl
                hx = self.hx_trl
                acc = c @ self.x_trl + d*in_
            elif position == 'trr':
                P = self.P_trr
                hx = self.hx_trr
                acc = c @ self.x_trr + d*in_

            K = P * np.transpose(C) * np.linalg.inv(C * P * np.transpose(C) + V)
            zk = hx + acc * T
            xk = hx + K * (zk - C * hx)
            p_buff = (np.eye(1) - K * C) * P
            hx = Phi * xk + Gamma * acc
            P = Phi * p_buff * np.transpose(Phi) + (Tau * W) * np.transpose(Tau)

            if position == 'tfl':
                self.P_tfl = P
                self.hx_tfl = hx
                self.x_tfl = a @ self.x_tfl + b * in_
            elif position == 'tfr':
                self.P_tfr = P
                self.hx_tfr = hx
                self.x_tfr = a @ self.x_tfr + b * in_
            elif position == 'trl':
                self.P_trl = P
                self.hx_trl = hx
                self.x_trl = a @ self.x_trl + b * in_
            elif position == 'trr':
                self.P_trr = P
                self.hx_trr = hx
                self.x_trr = a @ self.x_trr + b * in_

        elif index == 2: # COM
            acc = in_
            w1 = 0.7752
            v1 = -2.9322
            v2 = 4.9978

            V = np.array([[10**v1,0],
                         [0,10**v2]])
            W = 10**w1
            T = 0.01
            C = np.eye(2)
            Phi = np.array([[1,T],
                            [0,1]])
            Gamma = np.array([[0.5*T**2],[T]])
            Tau = np.array([[0.5*T**2],[T]])

            P = self.P_z
            hx = self.hx_z

            K = P * np.transpose(C) * np.linalg.inv(C * P * np.transpose(C) + V)
            zk = hx + vel * T
            xk = hx + K * (zk - C * hx)
            p_buff = (np.eye(1) - K * C) * P
            hx = Phi * xk + Gamma * vel
            P = Phi * p_buff * np.transpose(Phi) + (Tau * W) * np.transpose(Tau)

            self.P_z = P
            self.hx_z = hx

        elif index == 3: # angle position >> almost scalar calculation, so @ becomes * "fuxxing python!"
            vel = in_
            w1 = -0.7034
            v1 = 1.8197
            T = 0.01
            Phi = 1
            Gamma = T
            Tau = T
            C = np.eye(1)
            V = 10**v1
            W = 10**w1

            if position == 'phi':
                P = self.P_phi
                hx = self.hx_phi
            elif position == 'theta':
                P = self.P_theta
                hx = self.hx_theta

            K = P * np.transpose(C) * np.linalg.inv(C * P * np.transpose(C) + V)
            zk = hx + vel * T
            xk = hx + K * (zk - C * hx)
            p_buff = (np.eye(1) - K * C) * P
            hx = Phi * xk + Gamma * vel
            P = Phi * p_buff * np.transpose(Phi) + (Tau * W) * np.transpose(Tau)

            if position == 'phi':
                self.P_phi = P
                self.hx_phi = hx
            elif position == 'theta':
                self.P_theta = P
                self.hx_theta = hx

        return xk

    def ekf(self,eng,Z,hx,P,u):
        # print(Z)
        # print(hx)
        # print(P)
        # print(u)
        # print(type(Z),type(hx),type(P),type(u))
        eng.EKF_step(double_m(Z),double_m(hx),double_m(P),double_m(u))
        self.P_total = np.array(eng.eval('P'))
        self.hx_total = np.array(eng.eval('Hx'))
        self.Xk = np.array(eng.eval('Xk'))
        # print(np.array(self.Xk))
        return self.Xk

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
                writer.writerow(['episode', 'step', 'reward', 'FL_acc','FL_vel','FL_pos','FR_acc','FR_vel','FR_pos','RL_acc','RL_vel','RL_pos','RR_acc','RR_vel','RR_pos','TFL_acc','TFL_vel','TFL_pos','TFR_acc','TFR_vel','TFR_pos','TRL_acc','TRL_vel','TRL_pos','TRR_acc','TRR_vel','TRR_pos','body_acc','body_vel','body_pos','phi_acc','phi_vel','phi_pos','theta_acc','theta_vel','theta_pos','u_FL','u_FR','u_RL','u_RR','car_vel_l','car_vel_r','road_input_l','road_input_r'])
                writer.writerow(row)
