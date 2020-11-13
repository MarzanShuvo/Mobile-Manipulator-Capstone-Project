"""author: Marzan Alam
   Date: 13-11-2020"""
import modern_robotics as mr
import numpy as np
from math import cos, sin
import matplotlib.pyplot as plt
import logging

logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.INFO)
np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)

l = 0.47/2
w = 0.3/2
r = 0.0475

add_err = np.zeros(6)

h_zero = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]]) #h(theta)
Tb_0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]]) #Tb0
M0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])  #M0e
Blist = np.array([[0, 0, 1, 0, 0.033, 0],[0, -1,0, -0.5076, 0 ,0],[0, -1, 0, -0.3526, 0 ,0],[0, -1, 0, -0.2176, 0 ,0],[0, 0, 1, 0, 0, 0]]).T

for_next_state = [] #for saving the chassis information
error1=[]
error2 = []
error3 = []
error4 = []
error5 = []
error6 = []
error = []


#Next Stage function
def NextState(config, control, dt, speed, g):
    
    phi, x, y, a1, a2, a3, a4, a5, w1, w2, w3, w4, _ = config[0]
    a1dot, a2dot, a3dot, a4dot, a5dot, w1dot, w2dot, w3dot, w4dot = control[0]
    tsb = np.array([[cos(phi), -sin(phi), 0, x], [sin(phi), cos(phi), 0, y], [0, 0, 1, 0.0963], [0, 0, 0, 1]])
    theta_del = np.array([[w1dot, w2dot, w3dot, w4dot]])*dt
    Vb = np.dot(h_zero, theta_del.T)*r/4
    Vb6 = np.array([[0, 0, Vb[0][0], Vb[1][0], Vb[2][0], 0]]).T
    Vb6_vec = mr.VecTose3(Vb6)
    Tb_bk = mr.MatrixExp6(Vb6_vec)
    Tsb = np.dot(tsb, Tb_bk)
    #new orientation of robot
    phi_ = np.arcsin(Tsb[1][0])
    x_ = Tsb[0][3]
    y_ = Tsb[1][3]

    #new arm joint angle
    a1 = a1+a1dot*dt
    a2 = a2+a2dot*dt
    a3 = a3+a3dot*dt
    a4 = a4+a4dot*dt
    a5 = a5+a5dot*dt

    #new arm joint angle
    w1 = w1+w1dot*dt
    w2 = w2+w2dot*dt
    w3 = w3+w3dot*dt
    w4 = w4+w4dot*dt

    #used for checking if phi is an instance of numpy arrays
    if(isinstance(phi_, np.ndarray)):
        new_config = [phi_[0], x_[0], y_[0], a1, a2, a3, a4, a5, w1, w2, w3, w4, g]
        for_next_state.append([phi_[0], x_[0], y_[0], a1, a2, a3, a4, a5, w1, w2, w3, w4, g])
    else:
        
        new_config = [phi_, x_, y_, a1, a2, a3, a4, a5, w1, w2, w3, w4, g]
        for_next_state.append([phi_, x_, y_, a1, a2, a3, a4, a5, w1, w2, w3, w4, g])

    return new_config #next state return value


#for trajectory generation
def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k):
    #inital to standoff position
    Tse_s = np.dot(Tsc_i, Tce_s)
    N = 2*k/0.01
    traj = mr.CartesianTrajectory(Tse_i, Tse_s, 2, N, 5)
    for i in range(int(N)):
        final.append([traj[i][0][0], traj[i][0][1], traj[i][0][2], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], traj[i][2][1], traj[i][2][2], traj[i][0][3], traj[i][1][3], traj[i][2][3], 0])
    
    #standoff to grasp
    Tse_g = np.dot(Tsc_i, Tce_g)
    N = 2*k/0.01
    traj = mr.CartesianTrajectory(Tse_s, Tse_g, 2, N, 5)
    for i in range(int(N)):
        final.append([traj[i][0][0], traj[i][0][1], traj[i][0][2], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], traj[i][2][1], traj[i][2][2], traj[i][0][3], traj[i][1][3], traj[i][2][3], 0])
    
    #closing the grasp
    List = final[-1]
    closed = []
    for i in range(len(List)-1):
        closed.append(List[i])
    closed.append(1)

    for i in range(100):
        final.append(closed)

    #grasp to standoff
    N = 2*k/0.01
    traj = mr.CartesianTrajectory(Tse_g, Tse_s, 2, N, 5)
    for i in range(int(N)):
        final.append([traj[i][0][0], traj[i][0][1], traj[i][0][2], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], traj[i][2][1], traj[i][2][2], traj[i][0][3], traj[i][1][3], traj[i][2][3], 1]) 

    #standoff to final stand off
    N = 5*k/0.01
    Tse_fs = np.dot(Tsc_f, Tce_s)
    traj = mr.CartesianTrajectory(Tse_s, Tse_fs, 3, N, 5)
    for i in range(int(N)):
        final.append([traj[i][0][0], traj[i][0][1], traj[i][0][2], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], traj[i][2][1], traj[i][2][2], traj[i][0][3], traj[i][1][3], traj[i][2][3], 1])
    
    #final standoff to final grasp
    N = 2*k/0.01
    Tse_fg = np.dot(Tsc_f, Tce_g)
    traj = mr.CartesianTrajectory(Tse_fs, Tse_fg, 2, N, 5)
    for i in range(int(N)):
        final.append([traj[i][0][0], traj[i][0][1], traj[i][0][2], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], traj[i][2][1], traj[i][2][2], traj[i][0][3], traj[i][1][3], traj[i][2][3], 1])
    
    #opening the grasp
    List = final[-1]
    opened = []
    for i in range(len(List)-1):
        opened.append(List[i])
    opened.append(0)

    for i in range(100):
        final.append(opened)

    #final grasp to final standoff
    N = 2*k/0.01
    traj = mr.CartesianTrajectory(Tse_fg, Tse_fs, 2, N, 5)
    for i in range(int(N)):
        final.append([traj[i][0][0], traj[i][0][1], traj[i][0][2], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], traj[i][2][1], traj[i][2][2], traj[i][0][3], traj[i][1][3], traj[i][2][3], 0])


#for feedback_control
def FeedbackControl(Xd, Xd_next, X, Kp, Ki, dt, Jarm, Jbase):
    global add_err
    Vd_se3 = mr.MatrixLog6(np.dot(mr.TransInv(Xd), Xd_next))*(1/dt)
    Vd = mr.se3ToVec(Vd_se3)
    x_in_xd = np.dot(mr.TransInv(X), Xd) #X^-1*X_d
    X_error_se3 = mr.MatrixLog6(x_in_xd)
    X_error = mr.se3ToVec(X_error_se3)
    adjoint = mr.Adjoint(x_in_xd)
    forward_control = np.dot(adjoint,Vd) #Forward controller term
    proportional = np.dot(Kp, X_error) #proportional controller term
    add_err  = add_err+X_error*dt  #for integral error adding
    integral = np.dot(Ki, add_err) 
    V = forward_control+proportional+integral
    Je = np.append(Jbase, Jarm, axis=1)
    velocity = np.dot(np.linalg.pinv(Je, rcond=1e-4), V)
    return velocity, X_error

#returning value of X, Jarm, Jbase
def Essential_function(phi, x, y, a1, a2, a3, a4, a5):
    Tsb_q = np.array([[cos(phi), -sin(phi), 0, x], [sin(phi), cos(phi), 0, y], [0, 0, 1, 0.0963], [0, 0, 0, 1]])
    thetalist = np.array([a1, a2, a3, a4, a5])
    T0e_theta = mr.FKinBody(M0e, Blist, thetalist)
    x_q_theta = np.dot(np.dot(Tsb_q, Tb_0), T0e_theta)
    jarm = mr.JacobianBody(Blist, thetalist)
    F = np.array([[0, 0, 0, 0], [0, 0, 0, 0],[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1], [0, 0, 0, 0]])
    jbase = np.dot(mr.Adjoint(np.dot(mr.TransInv(T0e_theta), mr.TransInv(Tb_0))), F)

    return x_q_theta, jarm, jbase #X, Jb, Jbase is return from here




Tse_initial = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
Tsc_initial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tsc_goal = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tce_standoff = np.array([[-0.7071, 0 , 0.7071, 0],[0,1,0,0],[-0.7071, 0, -0.7071, 0.2],[0,0,0,1]])
Tce_grasp = np.array([[-0.7071, 0 , 0.7071, 0.02],[0,1,0,0],[-0.7071, 0, -0.7071, -0.02],[0,0,0,1]])
k = 1

final = [] # to save trajectory after generation
TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_goal, Tce_grasp, Tce_standoff, k)
logging.info('Saving Trajectory csv file')
np.savetxt("/media/marzan/data storage/Robotics/all_code/final_project/trajectory.csv", final, delimiter=",")
dt = 0.01
phi=-np.pi/5
x=0.1
y=0.2
w1=0
w2=0
w3=0
w4=0
a1=0
a2=0
a3=0.2
a4=-1.6
a5=0
Kp = 140 * np.identity(6) #proportional gain
Ki = 0 * np.identity(6)  #Integral gain
speed = np.array([[1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]])
config = [[phi, x, y, a1, a2, a3, a4, a5, w1, w2, w3, w4,0]]
for_next_state.append([phi, x, y, a1, a2, a3, a4, a5, w1, w2, w3, w4,0])

for i in range(len(final)-1):
    #finding current end-effector reference configuration, Jarm/Jb, J base from Essential_function
    X, Jarm, Jbase = Essential_function(phi, x, y, a1, a2, a3, a4, a5)
    #end-effector reference configuration
    Xd = np.array([[final[i][0],final[i][1],final[i][2],final[i][9]],
                   [final[i][3],final[i][4],final[i][5],final[i][10]],
                   [final[i][6],final[i][7],final[i][8],final[i][11]],
                   [0,0,0,1]])
    #end-effector reference configuration at the next timestep
    Xd_Next = np.array([[final[i+1][0],final[i+1][1],final[i+1][2],final[i+1][9]],
                   [final[i+1][3],final[i+1][4],final[i+1][5],final[i+1][10]],
                   [final[i+1][6],final[i+1][7],final[i+1][8],final[i+1][11]],
                   [0,0,0,1]])
    #velocity twist and error value
    v, err = FeedbackControl(Xd, Xd_Next, X, Kp, Ki, dt, Jarm, Jbase)
    #control value
    control = np.array([np.append(v[-5:],v[:4])])
    #gripper orientation
    g = final[i][-1]
    #next position of chassis
    new_con = NextState(np.array([for_next_state[-1]]), control, dt, speed, g)
    #extracting phi,x,y,a1,a2,a3,a4,a5
    phi, x, y, a1, a2, a3, a4, a5, _, _, _, _,_ = new_con
    #adding each error separately for plotting error
    error1.append(err[0])
    error2.append(err[1])
    error3.append(err[2])
    error4.append(err[3])
    error5.append(err[4])
    error6.append(err[5])
    #for creating csv file of error
    error.append([err[0], err[1], err[2], err[3], err[4], err[5]])
#saving data
logging.info('Generating animation csv file.')
np.savetxt("/media/marzan/data storage/Robotics/all_code/final_project/error.csv", error, delimiter=",")
np.savetxt("/media/marzan/data storage/Robotics/all_code/final_project/chassis.csv", for_next_state, delimiter=",")
#for plotting data
logging.info('Writing error plot data.')
t=np.arange(0,16.99,0.01)
plt.plot(t,error1,label='error1')
plt.plot(t,error2,label='error2')
plt.plot(t,error3,label='error3')
plt.plot(t,error4,label='error4')
plt.plot(t,error5,label='error5')
plt.plot(t,error6,label='error6')
plt.legend()
plt.title("Error Plot")
plt.show()

logging.info('Done.')
