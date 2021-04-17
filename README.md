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

**[SDCE ND] Control and Trajectory Tracking for Autonomous Vehicles (Short Demo with Plots)<br/>
https://youtu.be/GwLt8-gqQ4A**
![Short demo with plots](/plots/short_demo_with_plots.png)

I generated the following plots by using a Python script I programmed:

```
python plot_pid_info.py
```

As you can see in the video, the car experiences some turbulence at the beginning, due to the nature of the PID controllers that I will explain later. In brief, the car experiences some turbulence when the car is behind the waypoints. As soon as the car reaches the waypoints, it gets more control over the situation. The car also experienced an offset when turning the corner at the end of the street. After that, the car moves smoothly and in a controlled way.

**Steering Plot**<br/>
![Steering Plot](/plots/steering.png)

In the Steering Plot, there are 2 curves: The Error Steering in blue and the Steering Output in orange. The Error Steering is the difference between the current steering and the desired steering suggested by the vectorial field I describe below. Basically, the vectorial field suggests the steering should be the flow direction from the average waypoint to the first (last) waypoint. The vectorial field also creates an ortonormal base from such flow direction and suggest a steering compensation if the projected position of the car is in the left or right cuadrants of such ortonormal base.

The Error Steering is somewhat low with few disturbances in the conflicting parts of the video: The beginning where the waypoints left behind the car and the corner where the car turned with an offset. The Steering Output is proportional to the Error Steering, due to proportional term of the PID controller. However, they are not equal because the derivative term of the PID controller prevents the car from overshooting the proportional reaction to the error. The integral term of the PID controller is small and we cannot notice its influence in the graph. However, if the car has a small drift due to hits and damage, the integrative term will alleviate such small problems. If the car has no problems, the integral error will have a mean close to zero, that is, positive errors and negative errors will cancel out, and the integral term will have no influcence.

The big bump near the iteration 100 occurs when the car turns right and there is a offset. The car compensates the offset in a proportionate way but it overshoots a little bit. And it keeps compensating and overshooting a little bit until it recovers the control fully. Throughout the small trajectory, there are small bumps when the car turns a little bit. When the car drives straight, it recovers the control fully.

**Throttle Plot**<br/>
![Throttle Plot](/plots/throttle.png)

In the Throttle Plot, there are 3 curves: The Error Throttle in blue, the Throttle Output in green, and the Brake Output in orange. The Error Throttle is the difference between the current speed and the desired speed suggested by the vectorial field I describe below. Basically, the vectorial field suggests the speed should be the average speed of the average waypoint. The vectorial field also creates an ortonormal base from the flow direction and suggest a speed compensation if the projected position of the car is in the ahead or behind cuadrants of such ortonormal base.

The Error Throttle is somewhat low with small disturbances (up to 0.5) throughout the trajectory. I admit the PID controller for throttle and brake is not so controlled. Why? Due to the vectorial field I created. It's an average and thus inexact in nature. However, the solution is quite robust, overall. 

The Throttle Output is proportional to the Error Throttle, due to proportional term of the PID controller. However, they are not equal because the derivative term of the PID controller prevents the car from overshooting the proportional reaction to the error. The integral term of the PID controller is small and we cannot notice its influence in the graph. However, if the car has a small drift due to hits and damage, the integrative term will alleviate such small problems.

One thing to notice is that both the Throttle Output and the Brake Output are positive. And the Error Throttle can be positive or negative. When the Error Throttle is positive, the Throttle Output shows a proportionate and positive reaction; and the Brake Output is zero. And when the Error Throttle is negative, the Brake Output shows a proportionate, smaller, and positive reaction; and the Throttle Output is zero. The Brake Output shows a smaller reaction because the brake is stronger than the accelerator.

## What is the effect of the PID according to the plots, how each part of the PID affects the control command?

I repeat:

The Steering Output is proportional to the Error Steering, due to proportional term of the PID controller. However, they are not equal because the derivative term of the PID controller prevents the car from overshooting the proportional reaction to the error. The integral term of the PID controller is small and we cannot notice its influence in the graph. However, if the car has a small drift due to hits and damage, the integrative term will alleviate such small problems. If the car has no problems, the integral error will have a mean close to zero, that is, positive errors and negative errors will cancel out, and the integral term will have no influcence.

The Throttle Output is proportional to the Error Throttle, due to proportional term of the PID controller. However, they are not equal because the derivative term of the PID controller prevents the car from overshooting the proportional reaction to the error. The integral term of the PID controller is small and we cannot notice its influence in the graph. However, if the car has a small drift due to hits and damage, the integrative term will alleviate such small problems.

One thing to notice is that both the Throttle Output and the Brake Output are positive. And the Error Throttle can be positive or negative. When the Error Throttle is positive, the Throttle Output shows a proportionate and positive reaction; and the Brake Output is zero. And when the Error Throttle is negative, the Brake Output shows a proportionate, smaller, and positive reaction; and the Throttle Output is zero. The Brake Output shows a smaller reaction because the brake is stronger than the accelerator.

## How would you design a way to automatically tune the PID parameters? This is an open question, the coherence and justification of the answer is valued. 

As you can see, I tuned the PID parameters by trials and errors, the oldest optimization method. I also used my own experience with PID controllers since I took the course "Artificial Intelligence for Robotics" in my masters in Georgia Tech, the OMS CS. So, I guessed coherent parameters and saved some time. I performed countless experiments trying to understand not only the PID parameters but also the other parameters of my vectorial fields.

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

How would you design a way to automatically tune the PID parameters? Obviously, we need to speed up the simulation. Perhaps, by turning off the graphical part of the simulation, it could be sped up while preserving its realism and accuracy. In a simulation, time can be stretched and compressed without modifying the outcome. In that way, we could perform tons of experiments while varying the parameters. We should select the parameters whose RMSE is smaller. We can start from multiple hyperpoints and see where they ultimately land, in local optima. From all local optima, we can select the best optimum. That's called evolution by natural selection. **Important note: This process should be done automatically, not manually.**

How can we slightly vary the PID parameters in an automatic way? One clever way to vary the PID parameters is to use the Twiddle algorithm. In this video lecture, Prof. Sebastian Thrun explains this optimization process:<br/>
**Twiddle Algorithm - Artificial Intelligence for Robotics<br/>
https://www.youtube.com/watch?v=2uQ2BSzDvXs**

As the instructor suggested, another idea is to create a feedback loop system where the PID is slightly changing the parameters depending on the performances or error metrics. At the meta level, PID is an open loop controller <https://en.wikipedia.org/wiki/Open-loop_controller> because it does not have error feedback. If we want to minimize the errors of the PID controller, we need to close the loop by providing error feedback. So, we can slightly change the PID parameters and select those parameters which generate lesser errors. The best PID parameters are saved in a file. So, the PID controller gets better and better with more experiences. Optimization is a machine learning technique.

These 2 techniques proposed are very similar. The first technique is done with entire trajectories. And the second technique is done with smaller trajectories in realtime with a feedback loop system. Unfortunately, there are no video lectures explaining the details of the second technique, yet. There are no video lectures in the "Lesson 6. Control" of the Self Driving Car Engineer Beta Nanodegree Program <https://classroom.udacity.com/nanodegrees/nd013beta/parts/60d6f38c-c128-4d1c-997f-b5b762513b33>

## PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller? Find at least 2 pros and cons for model free versus model based.

Pros:
1. Model-free PID controllers are already conceived. And we just need to tune them. So, we skip the modeling part.
2. Model-free PID controllers understand the nature of differential equations. Because they adapt proportionally to the error; prevent overshooting through the derivative term; and integrate the constant errors of the system. Differential equations are everywhere in the universe and in nature. So, model-free PID controllers can be applied to a big variety of control problems, in spite of its simplicity.
3. The other approach, modeling the car and its dynamics requires an advanced understanding of differential equations and how to solve them. Sometimes models and their differential equations can be so complex that only a handful of people in the world can solve them.

Cons:
1. Sometimes tuning the parameters until getting perfect results is hard, if not impossible.
2. Metaoptimizing the parameters while exploring all the possibilities takes a lot of time. So, accelerating the time of simulations while turning graphics off becomes an option to overcome this issue.
3. PID controllers deal with continuous problems. Problems with discontinuities and singularities cannot be solved by using PID controllers.

## (Optional) What would you do to improve the PID controller? This is an open question, the coherence and justification of the answer is valued. 

We need to speed up the simulation. Perhaps, by turning off the graphical part of the simulation, it could be speed up while preserving its realism and accuracy. In a simulation, time can be stretched and compressed without modifying the outcome. In that way, we could perform tons of experiments while varying the parameters. We should select the parameters whose RMSE is smaller. We can start from multiple hyperpoints and see where they ultimately land, in local optima. From all local optima, we can select the best optimum. That's called evolution by natural selection.

My approach of the vectorial fields with the average waypoint is way better than my other approach of the vectorial fields with the closest waypoint, due to the predictive nature of the first approach. However, I have a new approach to test, if I had time, which is based on other vectorial fields with the smooth trajectories formed by the closest waypoints. Also, I didn't test having a memory of the waypoints. So that, if the car is way behind the waypoints, the car can recover without losing the track.

# Mathematical explanation of the vectorial fields

In dynamical systems and differential equations, PID controllers are easier to calibrate when goal states are based on vectorial fields. So, I programmed 2 vectorial fields to program the 2 PID controllers for the steering and the throttle of self-driving cars.

The Error Steering is the difference between the current steering and the desired steering suggested by the vectorial field I describe below. Basically, the vectorial field suggests the steering should be the flow direction from the average waypoint to the first (last) waypoint. The vectorial field also creates an ortonormal base from such flow direction and suggest a steering compensation if the projected position of the car is in the left or right cuadrants of such ortonormal base. The recommended steering plus the steering compensation creates a powerful attractor that draws the car in the right track. The car needs to predict the future recommended steering, not the direction of the closest waypoint in the present. Predicting the future produced better results.

The Error Throttle is the difference between the current speed and the desired speed suggested by the vectorial field I describe below. Basically, the vectorial field suggests the speed should be the average speed of the average waypoint. The vectorial field also creates an ortonormal base from the flow direction and suggest a speed compensation if the projected position of the car is in the ahead or behind cuadrants of such ortonormal base. The recommended speed plus the speed compensation creates a somewhat powerful attractor that draws the car toward the average waypoint, helping to keep the car on track.

**Vectorial fields for steering and throttle:**<br/>
![Vectorial fields for steering and throttle](/images/vectorial_fields.png)

In this way, I compute the ortonormal base and the projection of the car location onto the ortonormal base:

```
      // computes the ortonormal base
      Vector2D *direction = last_point->subtract(central_point);
      i = direction->unitary();
      j = new Vector2D(-i->y, i->x);
      // computes the projection of the car location onto the ortonormal base
      Vector2D *d = location->subtract(central_point);
      projection = new Vector2D(d->dot_product(i), d->dot_product(j));
```

And this is how I compute the steering compensation and the speed compensation:

```
      double steering = correct_angle(direction->angle());
      double steering_compensation = correct_angle(compute_steering_compensation());
      double speed = min(avg_speed, 3);
      double speed_compensation = compute_speed_compensation();

...

  double compute_steering_compensation() {
    double max_angle = M_PI * 0.25;
    double angle_compensation = -projection->y * 0.5; //0.25; //0.75; // * 0.5;
    if(angle_compensation > max_angle) angle_compensation = max_angle;
    if(angle_compensation < -max_angle) angle_compensation = -max_angle;
    return angle_compensation;
  }
  
  double compute_speed_compensation() {
    double max_speed = 1.5; //1;
    double offset = 0; //0.5; //-0.5; //-1;
    double speed_compensation = -(projection->x - offset) * 0.15; //0.2; //0.1;
    if(speed_compensation > max_speed) speed_compensation = max_speed;
    if(speed_compensation < -max_speed) speed_compensation = -max_speed;
    //if(speed_compensation < 0) speed_compensation *= 2;
    return speed_compensation;
  }
```

Notice that if the car is ahead of the first (last) waypoint or the average speed is zero or there are no spirals, the car should stop.

```
    if(abs(avg_speed) < ALMOST_ZERO || n_spirals == 0) {
    //if(any_waypoint_stopped) {
      return recommended_to_stop(current_angle, current_speed);
    } else {
    
    ...
    
      if(projection->x > direction->magnitude()) 
        return recommended_to_stop(current_angle, current_speed);
```

# Longer Demo (11 minutes)

**[SDCE ND] Control and Trajectory Tracking for Autonomous Vehicles (Demo 2)<br/>
https://youtu.be/Tofv9COgiks**
![Demo](/images/demo.png)

# Known Bugs

These bugs are a big source of headaches. I hope you will pay attention to them and will correct them. So, students won't experience the frustration and blockages I had.

## The car location is unknown

The car location is unknown. So, I had to send it in `simulatorAPI.py` and I had to receive it in `main.cpp`:

`simulatorAPI.py`
```
                t = world.player.get_transform()
                location_x = t.location.x
                location_y = t.location.y
                location_z = t.location.z
    
                ws.send(json.dumps({'traj_x': x_points, 'traj_y': y_points, 'traj_v': v_points ,'yaw': _prev_yaw, "velocity": velocity, 'time': sim_time, 'waypoint_x': waypoint_x, 'waypoint_y': waypoint_y, 'waypoint_t': waypoint_t, 'waypoint_j': waypoint_j, 'tl_state': _tl_state, 'obst_x': obst_x, 'obst_y': obst_y, 'location_x': location_x, 'location_y': location_y, 'location_z': location_z } ))
```

`main.cpp`
```
          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];
```

## Spirals and waypoints disappear and reappear in a discontinuous way

Spirals and waypoints disappear and reappear in a discontinuous way, pointing toward a totally different direction, causing disorientation in the self-driving car. This error is quite common and sporadic. I thought it was caused when the car gets apart from the waypoints. But I experienced the same bug when the car was quite aligned with the waypoints. So, I think this bug is yours, not mine. And you should correct this bug, which is super frustrating for students. If this bug didn't exist, my car could drive in an indefinite way, since my vectorial field solution is quite robust and can recover from almost any continuous problem, except from big discontinuities in the waypoints. I think this error occurs in junctions. So, pay attention to the code for junctions and also to the code for generating spirals.

**[SDCE ND] Bug report: Spirals and waypoints disappear and reappear in a discontinuous way<br/>
https://youtu.be/q70djixQsHY**
![Spirals disappear](/images/spirals_disappear.png)

## Division by zero when waypoints gather to the same location

A division by zero occurs when waypoints gather to the same location. I already corrected this bug with this Python code:

`simulatorAPI.py`
```
                else:
                    _prev_yaw = yaw
                    D = velocity * delta_t
                    d_interval = math.sqrt( (way_points[1].location.x - way_points[0].location.x )**2 + (way_points[1].location.y - way_points[0].location.y )**2  )
                    while d_interval < D and len(way_points) > 2:
                        D -= d_interval
                        way_points.pop(0)
                        v_points.pop(0)
                        d_interval = math.sqrt( (way_points[1].location.x - way_points[0].location.x )**2 + (way_points[1].location.y - way_points[0].location.y )**2  )
                    if abs(d_interval) < 1e-6:
                        #v_points[0] = v_points[0]
                        #way_points[0].location.x = way_points[0].location.x
                        #way_points[0].location.y = way_points[0].location.y
                        yaw = 0.
                    else:
                        ratio = D / d_interval # SUSPICIOUS
                        v_points[0] = ratio * (v_points[1]-v_points[0]) + v_points[0]
                        way_points[0].location.x = ratio * (way_points[1].location.x - way_points[0].location.x) + way_points[0].location.x
                        way_points[0].location.y = ratio * (way_points[1].location.y - way_points[0].location.y) + way_points[0].location.y
                        yaw = math.atan2(way_points[1].location.y-way_points[0].location.y, way_points[1].location.x-way_points[0].location.x)
```

## List index error when drawing spirals

A list index error when drawing spirals. So, I had to comment out the code for drawing spirals:

`simulatorAPI.py`
```
        # draw spirals
        height_plot_scale = 1.0
        height_plot_offset = 1.0
        """
        blue = carla.Color(r=0, g=0, b=255)
        green = carla.Color(r=0, g=255, b=0)
        red = carla.Color(r=255, g=0, b=0)
        for i in range(len(spirals_x)):
            previous_index = 0
            previous_speed = 0
            start = carla.Transform()
            end = carla.Transform()
            color = blue
            if i == spiral_idx[-1]:
                color = green
            elif i in spiral_idx[:-1]:
                color = red
            for index in range(1, len(spirals_x[i])):
                start.location.x = spirals_x[i][previous_index]
                start.location.y = spirals_y[i][previous_index]
                end.location.x = spirals_x[i][index]
                end.location.y = spirals_y[i][index]
                start.location.z = height_plot_scale * spirals_v[i][previous_index] + height_plot_offset + _road_height
                end.location.z =  height_plot_scale * spirals_v[i][index] + height_plot_offset + _road_height
                self.world.debug.draw_line(start.location, end.location, 0.1, color, .1)
                previous_index = index  
        """
```

## The game loop does not report exceptions thrown

At the end of the game loop, I added 3 lines to report the exceptions thrown in the game loop:

`simulatorAPI.py`
```
    except Exception as error:
        print('EXCEPTION IN GAME LOOP:')
        print(error)
    finally:

        print("key board interrupt, good bye")
        if world is not None:
            world.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        pygame.quit()        
```
