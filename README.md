# Control_and_Trajectory_Tracking

**Control and Trajectory Tracking for Autonomous Vehicles**<br/>
Self-Driving Car Engineer Nanodegree<br/>
https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013

In this project, you apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment (the vehicle with possible perturbations), you design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

# Installation

Download this github repository in your local drive: 
https://github.com/jckuri/Control_and_Trajectory_Tracking/archive/refs/heads/main.zip

Unzip the file `Control_and_Trajectory_Tracking-main.zip`
CD into the folder `Control_and_Trajectory_Tracking-main`
Run the script `copy_files.sh`:
```
sh copy_files.sh
```

The script `copy_files.sh` basically clones this project from the Udacity repository and copies all the files I modified into the corresponding folders inside the github repository in an automatic way:
```
git clone https://github.com/udacity/nd013-c6-control-starter.git
cp code/compile_pid_controller.sh nd013-c6-control-starter/project/
cp code/simulatorAPI.py nd013-c6-control-starter/project/
cp code/plot_pid_info.py nd013-c6-control-starter/project/
cp code/main.cpp nd013-c6-control-starter/project/pid_controller/
cp code/pid_controller.cpp nd013-c6-control-starter/project/pid_controller/
cp code/pid_controller.h nd013-c6-control-starter/project/pid_controller/
```

First, you must install Conda.
https://docs.conda.io/en/latest/

Install Python 3.7:
```
conda create -n SDC python=3.6
conda activate SDC
```

Install the Carla Simulator:
https://carla.org/
https://carla.readthedocs.io/en/latest/start_quickstart/

sh install_project.sh 


**Screenshot after successfully installing and running the default code without behavior:**<br/>
![Screenshot after successfully installing and running the default code](/images/screenshot_01.png)

# answers.txt

## Add the plots to your report and explain them (describe what you see)

[SDCE ND] Control and Trajectory Tracking for Autonomous Vehicles (Short Demo with Plots)<br/>
https://youtu.be/GwLt8-gqQ4A
![Short demo with plots](/plots/short_demo_with_plots.png)

Steering Plot<br/>
![Steering Plot](/plots/steering.png)

Throttle Plot<br/>
![Throttle Plot](/plots/throttle.png)


## What is the effect of the PID according to the plots, how each part of the PID affects the control command?


## How would you design a way to automatically tune the PID parameters? This is an open question, the coherence and justification of the answer is valued. 

```
  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/
  PID pid_steer = PID();
  double max_steer = 1.2; //0.25 * M_PI; // ORIGINAL
  //pid_steer.Init(0.2, 0.05, 0.05, max_steer, -max_steer, 50);
  //pid_steer.Init(0.15, 0.005, 0.05, max_steer, -max_steer, 10); // VERY GOOD!
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10); // GREAT!!!
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10); // GREAT!!! (11 MINUTES)
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10);
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10);
  //pid_steer.Init(0.3, 0.05, 0.3, max_steer, -max_steer, 10); // good turn?
  //pid_steer.Init(0.3, 0.1, 0.2, max_steer, -max_steer, 10); // good turn!
  //pid_steer.Init(0.3, 0.1, 0.2, max_steer, -max_steer, 10);
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10); // good control
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10); // great turn and timing
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10); 
  //pid_steer.Init(0.25, 0.1, 0.4, max_steer, -max_steer, 10); // latest before future point
  //pid_steer.Init(0.2, 0.01, 0.2, max_steer, -max_steer, 10); // very good but it crashes wall
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10); // 22 minutes!!!
  //pid_steer.Init(0.25, 0.05, 0.4, max_steer, -max_steer, 10); // perfect turn
  //pid_steer.Init(0.25, 0.02, 0.4, max_steer, -max_steer, 10); // good enough for speed control and turns
  //pid_steer.Init(0.25, 0.02, 0.4, max_steer, -max_steer); // ORIGINAL
  //pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer);
  //pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer);
  //pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer);
  pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer); //25 minutes!
    
  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/
  PID pid_throttle = PID();
  double max_throttle = 1; //0.75; //1; // ORIGINAL
  double max_break = -1; //-0.25; //-0.25; //-0.15; // ORIGINAL
  //pid_throttle.Init(1, 0.05, 0.0, max_throttle, max_break, 50);
  //pid_throttle.Init(1, 0.075, 0.01, max_throttle, max_break, 50);
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // GREAT!!!
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10);
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // GREAT!!! (11 MINUTES)
  //pid_throttle.Init(0.20, 0.05, 0.1, max_throttle, max_break, 10);
  //pid_throttle.Init(0.2, 0.05, 0.2, max_throttle, max_break, 10); // good turn but too much
  //pid_throttle.Init(0.1, 0.05, 0.1, max_throttle, max_break, 10); // too low
  //pid_throttle.Init(0.2, 0.1, 0.1, max_throttle, max_break, 10); // good turn but too much
  //pid_throttle.Init(0.2, 0.05, 0.1, max_throttle, max_break, 10); // good timing
  //pid_throttle.Init(0.2, 0.05, 0.1, max_throttle, max_break, 10); // great turn and timing
  //pid_throttle.Init(0.2, 0.05, 0.1, max_throttle, max_break, 10);
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // 22 minutes!!!
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // perfect turn
  //pid_throttle.Init(0.25, 0.02, 0.1, max_throttle, max_break, 10); // good enough for speed control and turns
  //pid_throttle.Init(0.25, 0.02, 0.1, max_throttle, max_break); // ORIGINAL
  //pid_throttle.Init(0.25, 0.02, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.3, 0.02, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.4, 0.02, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.35, 0.01, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.35, 0.01, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.25, 0.01, 0.1, max_throttle, max_break);
  //pid_throttle.Init(0.25, 0.01, 0.05, max_throttle, max_break);
  pid_throttle.Init(0.35, 0.01, 0.2, max_throttle, max_break); //25 minutes!
```

## PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller? Find at least 2 pros and cons for model free versus model based.


## (Optional) What would you do to improve the PID controller? This is an open question, the coherence and justification of the answer is valued. 

# Mathematical explanation of my vectorial fields

Vectorial fields for steering and throttle:<br/>
![Vectorial fields for steering and throttle](/images/vectorial_fields.png)

# Longer Demo (11 minutes)

[SDCE ND] Control and Trajectory Tracking for Autonomous Vehicles (Demo 2)<br/>
https://youtu.be/Tofv9COgiks
![Demo](/images/demo.png)
