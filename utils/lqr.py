import numpy as np

def computeLQR(env, action):
    action = (action + 1) / 2
    
    K = env.matlab_eng.LQR(env.A, env.B, float(action[0])+0.001, float(action[1])+0.001, float(action[2])+0.001, pow(10,float(action[3])))

    X = [
        env.dll.getStateZ3(),
        env.dll.getStateZ2(),
        env.dll.getStatePhi2(),
        env.dll.getStatePhi1(),
        env.dll.getStateTheta2(),
        env.dll.getStateTheta1(),
        env.dll.getStateTFL3(),
        env.dll.getStateTFL2(),
        env.dll.getStateTFR3(),
        env.dll.getStateTFR2(),
        env.dll.getStateTRL3(),
        env.dll.getStateTRL2(),
        env.dll.getStateTRR3(),
        env.dll.getStateTRR2(),
    ]

    # [u_fl, u_fr, u_rl, u_rr] = - np.matmul(K , np.transpose(X))
    return - np.matmul(K , np.transpose(X))