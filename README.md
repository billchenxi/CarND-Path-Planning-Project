# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Results:
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car simulation has achieved speed as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. To achieve this goal, the car simulation has successfully avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, and smoothly change lane when necessary. It took a little over 5 minutes to complete 1 loop without experiencing the total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.



### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Discussion

In this project, I used the same approaches in the project walktrhough for cold start, acceleration, speed limits, jerk avoidance and lane-keeping, spline and so on. I also consider the code from this repo: https://github.com/darienmt/CarND-Path-Planning-Project-P1/blob/master/src/main.cpp.
#### major change from the walkthrough code 
The instructor from the walkthrough video provided a lot of helps, but I have hard time to follow when there's no code file provided. The video is too dark and the font is too small. Hard to copy from the screen.

Anyway. the major contribution is I provided a more sophisticated lane change function and smooth out the transitions. 

##### Lane change part
In this part, I first check whether all the lane free or not, and then basing on the distance between the self-car and other cars and the lane speed detected from the sensor fusion, my code will decided whether to switch and which lane to switch to. 

```$xslt
 if( d<(2+4*lane+2) && d>(2+4*lane-2) ){
                            check_car_s += ((double)prev_size*time_step*check_speed); // if using previous points can project s value out
                            // check s values greater than mine and s gap
                            if((check_car_s > car_s) && ((check_car_s-car_s) < warn_dist)){
                                // Do some logic here, lower reference velocity so we don't crash into the car in front of use could also flag to try to change lanes.
                                // ref_vel = 29.5;
                                too_close = true;
                            }
                        }

                        // check whether lane is free?
                        if( (car_s-2*safe_dist)<check_car_s && (car_s+safe_dist)>check_car_s){
                            lane_free[check_lane] = false;
                        }

                        // change lane speed if the condition obtains
                        if( lane_speed[check_lane]>car_speed ){
                            lane_speed[check_lane] = car_speed;
                        }
                    }


                    // change to the best lane ********************
                    // check whether at least one lane is free.
                    // logic here is if all lanes are false, so as long as there's one is true, it will be false for all then ! will change to true.
                    change_lane = !all_of(lane_free.begin(), lane_free.end(), [](bool e){return e ==false;});

                    if(change_lane){
                        switch(lane){
                            // self_car is in the left lane.
                            case 0:
                                new_lane = lane_free[1] ? 1 : lane;
                                break;

                            // self_car is in the middle lane.
                            case 1:
                                if(lane_speed[0] > lane_speed[2]){
                                    if(lane_free[0]) new_lane = 0;
                                    else if(lane_free[2]) new_lane = 2;
                                }else{
                                    if(lane_free[2]) new_lane = 2;
                                    else if(lane_free[0]) new_lane = 0;
                                }
                            break;

                            // self_car is in the right lane
                            case 2:
                                new_lane = lane_free[1] ? 1 : lane;
                                break;
                        }
                    }

                    // avoid switch lane too fast
                    current_time = clock();

                    bool d_change = ((current_time - change_lane_timestamp)
                                     /10000) > 5.0; // s

                    /* Change lane condition
                     *  - there's a car in the front;
                     *  - changing lane is possible;
                     *  - new lane is a different one than current one;
                     *  - the changing lane was not happened recently, > 0.5s;
                     * */
                    if(too_close && change_lane && new_lane!=lane && d_change){
                        lane = new_lane; // change to lane
                        change_lane_timestamp = clock(); // save to timestep
                        // ref_vel -= .224;
                        for(int i=0; i< num_lanes; i++){
                            lane_speed[i] = 1000;
                        }
                        too_close = false;
                    }
//                    else if(ref_vel < 49.5){
//                        ref_vel += .224;
//                    }
//                    else if(ref_vel >= 49.5){
//                        ref_vel -= .224;
//                    }

                    // reset lane to be free
                    for(int i=0; i<num_lanes; i++){
                        lane_free[i] = true;
                    }
```

##### Smoothing the transition
The following part is just the replication of the code from the walkthrough, instead of hard code, I just write them into variable so it's easier to tune, but the walkthrough has already offered good value fo the variables, so I just use the those from the it. 

```$xslt
// control the velocity smoothness
                    double d_vel = 0.224;
                    double target_vel = 49.5;

                    // if there's a car in the front, slow down.
                    if(too_close){
                        ref_vel -= d_vel;
                    } else if(ref_vel < target_vel){
                        ref_vel += d_vel;
                    }
```
---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

