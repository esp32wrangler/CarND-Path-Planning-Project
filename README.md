# **Path Planning Project**


**The goals / steps of this project are the following:**
The goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The goal is to get the car to drive as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

This project uses the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). See README_orig.md for getting the environment up and running.

[//]: # (Image References)

[image1]: ./writeup_images/simulator.png "Simulator screenshot "

---

## Writeup / README

I build my control module in several iterations.

The first challenge was to get the car to drive around the track. I tried the naive approach of just iterating with the `s` position, and converting the location with the `getXY` function to x,y coordinates. Unfortunately getXY uses a simple linear interpolation between the map points provided, which caused sharp turns at each waypoint, and this movement exceeded the acceleration and jerk limits set by the simulator.

Then I tried creating a smooth interpolation of the waypoints with three parallel tracks for the three lanes. This proved challenging as this is a round course and I couldn't just feed the cordinates into a spline interpolation without creating overlapping ranges.

Finally I decided to interpolate just the upcoming path with spline. I fought a bit with the exact implementation of rotating the coordinate system so I got non-overlapping x coordinates for the spline interpolation. Finally I checked the Q&A page and found the formula I needed for a working rotation/translation.

Next was speed control. I implemented a simple algorithm using a target speed, a maximum permitted acceleration and an "urgency factor". The code will iteratively increase or decrease the current speed of the car when calculating each path point. The change speed (acceleration) will be determined by multiplying this "urgency factor" with the maximum permitted acceleration, which was chosen to be 9 m/s<sup>2</sup>, leaving 1 m/s<sup>2 room for acceleration due to other factors such as turns and rounding errors. 

The urgency is set to 0.5 (4.5 m/s<sup>2</sup> acceleration) when starting out or when a lane opens up. If we encounter a car in front of our car, the code calculates the necessary minimum deceleration to match the lead vehicle speed and sets the target speed and urgency factor accordingly to maximize comfort. 

The next challenge was to switch lanes. It turned out that this was surprisingly simple to implement, just changing the goal of the upcoming path to a different lane automatically gave me the curves needed for the lane change. Therefore I didn't need to use the jerk minimalization formula from the course.

The last challenge was to select the optimal lane. I tried several different approaches, but finally settled on a simple formula of looking at each lane for the next 60 meters, determining the minimum speed of cars in the lane in that range, and selecting the lane with the highest speed. The 60 meter lookahead was selected to minimize the chance of getting stuck behind two cars going at near the same speed. 60 m is enough to allow for the car to get around a slow car and then get back to the original lane. A longer distance gave a pessimistic outlook and prevented the car from shifting to a lane where there was a slow car in the distance. On the other hand anything shorter than that often caused the car to switch to a lane and then get stuck behind a slow car in that lane. This algorithm isn't very elegant, but it worked better than slightly more sophisticated approaches trying to predict a path between the vehicles, especially thanks to the drivers who kept accelerating and braking continuously, causing prediction complications. Probably it would be possible to get a planner working, but for now the simple approach works surprisingly well.


## Testing

I ran the simulation for many laps, and it proved quite stable. 

There is one situation that it doesn't handle yet: if a slow car comes into out lane within the braking distance, and the other lanes has a car nearby, the code realizes that we are going to crash into it, but instead of swerving to avoid the dangerous driver, it just triggers a deceleration stronger than 10 m/s<sup>2</sup>. Unfortunately this case is rare enough so debugging a better algorithm was more time consuming than my patience allowed. It would be nice if I could set up certain scenarios in the simulator (for example by being able to control two cars instead of just one).

There was one more situation that could use improvement: if, for example we are in the rightmost lane behind a slow car, and there is another slow car in the center lane, the right approach would be to slow down, get behind the car in the center lane and then switch to the leftmost lane. With the current lane selection algorithm it almost never happens, but when it does, it would be nice to have a "stuck mode" avoidance algorithm.

![image1]
