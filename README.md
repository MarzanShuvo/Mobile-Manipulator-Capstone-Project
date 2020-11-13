# Mobile Manipulation Capstone

## About:

A software that plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down.

## Steps:
    * youBot Kinematics Simulator and csv Output
    * Reference Trajectory Generation
    * FeedBack Control
    * Combine All

## Implementation:

### youBot Kinematics Simulator
`def NextState(config, control, dt, speed, g)`

#### INPUT :
 * config : current chassis configuration
 * control : desired velocity of arm joint and wheel
 * dt : timesteps
* speed: maximum limited speed
 * g : graspping state
    
#### OUTPUT : 
* Next chassis Configuration

### Reference Trajectory Generation
`def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k)`

#### INPUT :
* Tse_i : Initial end effector transformation matrix in S frame
* Tsc_i : Inital cube transformation matrix in S frame
* Tsc_f : Final cube transformation matrix in S frame
* Tce_g : The end-effector's configuration relative to the cube when it is grasping the cube
* Tce_s : The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
* k : The number of trajectory reference configurations per 0.01 seconds

#### OUTPUT :
* A csv file with the entire eight-segment reference trajectory
* output format : `r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state'

#### FeedBack Control
`def FeedbackControl(Xd, Xd_next, X, Kp, Ki, dt, Jarm, Jbase)`

#### INPUT :
* Xd : The current end-effector reference configuration
* Xd_next : The end-effector reference configuration at the next timestep in the reference trajectory
* X : The current actual end-effector configuration X
* Kp & Ki : The PI gain matrices Kp and Ki.
* dt : The timestep Δt between reference trajectory configurations.
* Jarm: Jacobian of arms
* Jbase : Jacobian of base

#### OUTPUT :
* velocity : Velocity command for reaching next steps
* X_error : Error 

#### Essential Function :
`def Essential_function(phi, x, y, a1, a2, a3, a4, a5)`
         
#### INPUT :
* phi : Rotation of the chassis
* x : x-axis orientation
* y : y-axis orientation
* a1 : First joint angle
* a2 : Second joint angle
* a3 : Third joint anlge
* a4 : Forth joint angle
* a5 : Fifth joint angle
            
#### OUTPUT: 
* x_q_theta : current joint configuration
* jarm : arm jacobian
* jbase : base jacobian

## For running the code:
Go to : `./code/script.py`
Run : `python3 script.py`

## Result:

### Error Plot

![errorplot](https://github.com/MarzanShuvo/Mobile-Manipulator-Capstone-Project/blob/main/results/Best/error.png)

### Simulation
The Picture of the simulation

![Video](https://github.com/MarzanShuvo/Mobile-Manipulator-Capstone-Project/blob/main/results/Best/simulation_.png)

video will be found in
`results/Best/simulation.mp4`
