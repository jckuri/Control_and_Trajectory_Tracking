# Control_and_Trajectory_Tracking

**Control and Trajectory Tracking for Autonomous Vehicles**<br/>
Self-Driving Car Engineer Nanodegree<br/>
https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013

In this project, you apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment (the vehicle with possible perturbations), you design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

# Installation Prerequisites

Install Ubuntu:<br/>
https://ubuntu.com/

Install make:<br/>
https://www.gnu.org/software/make/

Install GCC:<br/>
https://www.gnu.org/software/gcc/

Install Conda.<br/>
https://docs.conda.io/en/latest/

Install Python 3.7:<br/>
```
conda create -n SDC python=3.7
conda activate SDC
```

Install the Carla Simulator:<br/>
https://carla.org/<br/>
https://carla.readthedocs.io/en/latest/start_quickstart/<br/>

If this error appears when running the Carla Simulator, don't forget to correctly configure the variable `PYTHONPATH`.<br/>
ModuleNotFoundError: No module named 'carla'<br/>
https://github.com/carla-simulator/scenario_runner/issues/367#issuecomment-645187463<br/>
In my case, the variable `PYTHONPATH` is:<br/>
```
export PYTHONPATH="/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg"
```
But in your case, it could be a different folder.<br/>

If this error appears, please install the library `websocket`.<br/>
ModuleNotFoundError: No module named 'websocket'<br/>
https://stackoverflow.com/questions/47665760/no-module-named-websocket/47666357<br/>
```
pip install websocket
```

If the library `boost` is not installed and errors appear, please install the library `boost`.<br/>
https://stackoverflow.com/questions/12578499/how-to-install-boost-on-ubuntu<br/>
```
sudo apt-get install libboost-all-dev
```

If you `apt-get update` fails at some non-working repositories, you can follow these steps:<br/>
https://askubuntu.com/questions/91543/apt-get-update-fails-to-fetch-files-temporary-failure-resolving-error<br/>
Add a # to the beginning of the line to comment it out - for example<br/>
`#deb http:/archive.canonical.com/ natty backports`<br/>
Save and re-run:<br/>
`sudo apt-get update && sudo apt-get upgrade`<br/>
http://archive.ubuntu.com/ubuntu/<br/>

If the library `glog` is not installed, please install the library `glog`.<br/>
https://zoomadmin.com/HowToInstall/UbuntuPackage/libgoogle-glog-dev<br/>
```
sudo apt-get update -y
sudo apt-get install -y libgoogle-glog-dev
```

If the library `libgtest` is not installed and errors appear, please install the library `libgtest`.<br/>
No rule to make target '/usr/src/gtest/libgtest.a'<br/>
https://www.eriksmistad.no/getting-started-with-google-test-on-ubuntu/<br/>
```
sudo apt-get install libgtest-dev
```

If the libraries `blas` and `lapack` are not installed and errors appear, please install the libraries `blas` and `lapack`.<br/>
/usr/bin/ld: cannot find -llapack<br/>
https://askubuntu.com/questions/623578/installing-blas-and-lapack-packages<br/>
sudo apt-get install libblas-dev liblapack-dev<br/>

Kazam Screen Recorder is useful to record the animation in a video file.<br/>
https://itsfoss.com/kazam-screen-recorder/<br/>

Video Trimmer is useful to trim the video recording.<br/>
https://flathub.org/apps/details/org.gnome.gitlab.YaLTeR.VideoTrimmer<br/>

# Installation Steps

Download this github repository in your local drive:<br/>
https://github.com/jckuri/Control_and_Trajectory_Tracking/archive/refs/heads/main.zip

Unzip the file `Control_and_Trajectory_Tracking-main.zip`<br/>
CD into the folder `Control_and_Trajectory_Tracking-main`<br/>
Run the script `copy_files.sh`:<br/>
```
sh copy_files.sh
```

The script `copy_files.sh` basically clones this project from the Udacity repository and copies all the files I modified into the corresponding folders inside the github repository in an automatic way:<br/>
```
git clone https://github.com/udacity/nd013-c6-control-starter.git
cp code/compile_pid_controller.sh nd013-c6-control-starter/project/
cp code/simulatorAPI.py nd013-c6-control-starter/project/
cp code/plot_pid_info.py nd013-c6-control-starter/project/
cp code/main.cpp nd013-c6-control-starter/project/pid_controller/
cp code/pid_controller.cpp nd013-c6-control-starter/project/pid_controller/
cp code/pid_controller.h nd013-c6-control-starter/project/pid_controller/
```

CD into the project folder:<br/>
```
cd nd013-c6-control-starter/project
```

Run the script `install-ubuntu.sh`:<br/>
```
sh install-ubuntu.sh
```

CD into the folder `pid_controller`. Remove the folder `rpclib`. Clone the github repository `rpclib`. CD into the folder `rpclib`. Make the project `rpclib`.<br/>
```
cd pid_controller/
rm -rf rpclib
git clone https://github.com/rpclib/rpclib.git
cd rpclib
cmake .
make
```

Make the project `pid_controller`.<br/>
```
cd ..
cmake .
make
```

Run the Carla Simulator in another terminal:<br/>
```
cd /opt/carla-simulator/
sh CarlaUE4.sh
```

In the previous terminal where we made the projects, go to the folder `nd013-c6-control-starter/project`. Compile and run the project.<br/>
```
cd ..
sh compile_pid_controller.sh
sh run_main_pid.sh
```

Then the following window must appear indicating that you have successfully installed and run the project:<br/>

**Screenshot after successfully installing and running the default code without behavior**<br/>
![Screenshot after successfully installing and running the default code](/images/screenshot_01.png)

If this window does not appear and errors appear instead, it means that some **Installation Steps** or some **Installation Prerequisites** were skipped. Please, read the instructions carefully. Perhaps the instructions in the **Installation Prerequisites** are not comprehensive and there are some missing libraries. If so, google about how to install the missing libraries in the error messages.<br/>

# answers.txt

## Add the plots to your report and explain them (describe what you see)

Given that the visibility of the plots deteriorates when the demos are too long, I decided to limit the demo to 1:35 minutes. Here is the demo from which I extracted the data of the Steering Plot and the Throttle Plot.

[SDCE ND] Control and Trajectory Tracking for Autonomous Vehicles (Short Demo with Plots)<br/>
https://youtu.be/GwLt8-gqQ4A
![Short demo with plots](/plots/short_demo_with_plots.png)

As you can see in the video, the car experiments some turbulence at the beginning, due to the nature of the PID controllers that I will explain later. In brief, the car experiments some turbulence when the car is behind the waypoints. As soon as the car reaches the waypoints, it gets more control over the situation. The car also experienced an offset when turning the corner at the end of the street. After that, the car moves smoothly and in a controlled way.

Steering Plot<br/>
![Steering Plot](/plots/steering.png)

In the Steering Plot, there are 2 curves: The Error Steering in blue and the Steering Output in orange. The Error Steering is the difference between the current steering and the desired steering suggested by the vectorial field I describe below. Basically, the vectorial field suggests the steering should be the flow direction from the average waypoint to the first (last) waypoint. The vectorial field also creates an ortonormal base from such flow direction and suggest a steering compensation if the projected position of the car is in the left or right cuadrants of such ortonormal base.

The Error Steering is somewhat low with few disturbances in the conflicting parts of the video: The beginning where the waypoints left behind the car and the corner where the car turned with an offset. The Steering Output is proportional to the Error Steering, due to proportional term of the PID controller. However, they are not equal because the derivative term of the PID controller prevents the car from overshotting the proportional reaction to the error. The integral term of the PID controller is small and we cannot notice its influence in the graph. However, if the car has a small drift due to hits and damage, the integrative term will alleviate such small problems. If the car has no problems, the integral error will have a mean close to zero, that is, positive errors and negative errors will cancel out, and the integral term will have no influcence.

The big bump near the iteration 100 occurs when the car turns right and there is a offset. The car compensates the offset in a proportionate way but it overshoots a little bit. And it keeps compensating and overshooting a little bit until it recovers the control fully. Throughout the small trajectory, there are small bumps when the car turns a little bit. When the car drives straight, it recovers the control fully.

Throttle Plot<br/>
![Throttle Plot](/plots/throttle.png)

In the Throttle Plot, there are 3 curves: The Error Throttle in blue, the Throttle Output in green, and the Brake Output in orange. The Error Throttle is the difference between the current speed and the desired speed suggested by the vectorial field I describe below. Basically, the vectorial field suggests the speed should be the average speed of the average waypoint. The vectorial field also creates an ortonormal base from the flow direction and suggest a speed compensation if the projected position of the car is in the ahead or behind cuadrants of such ortonormal base.

The Error Throttle is somewhat low with few disturbances throughout the trajectory. I admit the PID controller for throttle and brake is not so controlled. Why? Due to the vectorial field I created. It's an average and thus inexact in nature. However, the solution is quite robust overall. 

The Throttle Output is proportional to the Error Throttle, due to proportional term of the PID controller. However, they are not equal because the derivative term of the PID controller prevents the car from overshotting the proportional reaction to the error. The integral term of the PID controller is small and we cannot notice its influence in the graph. However, if the car has a small drift due to hits and damage, the integrative term will alleviate such small problems.

One thing to notice is that both the Throttle Output and the Brake Output are positive. And the Error Throttle can be positive or negative. When the Error Throttle is positive, the Throttle Output shows a proportionate and positive reaction; and the Brake Output is zero. And when the Error Throttle is negative, the Brake Output shows a proportionate, smaller, and positive reaction; and the Throttle Output is zero. The Brake Output shows a smaller reaction because the brake is stronger than the accelerator.

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
