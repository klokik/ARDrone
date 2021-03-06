#Term paper

# Topic: Using optical markers for short range navigation

##Contents

## (Part 1) Introduction

### Problem identification
### Relevance
{Brief outline of robotics importance}
* product delivery
* ...

### "Branches of science"
* Geometry
* Linear algebra
* Mechanics
* Compuer science
* Simulation

## (Part 2) Existing approaches

{A diagram of relations should be located here}
* Global
  * Sattelite (GPS/GNSS/etc)
  * Stars
* Regional
  * Cellular networks
  * Wi-Fi
  * etc.

* Local
  * Vision
  * Inertial
  * Magnetic
  * Range scanning

* Inertial
* Radio
  * RSSI
  * Distance
  * Triangulation

* Vision
  * (Star based navigation at space)
  * Marker based
  * Depth map reconstruction
* Sonar
* LIDAR (Velodyne)

* Combined

{Short description, Pros&cons of the most relevant ones}

## (Part 3) Outline of the approach
There are two main tasks "that should be completed for accurate model pose estimation. The first one is to get accurate pose from the image input. The natural question that arises: how fast do we need to sample our location data to keep model stable in the air and move fast enought on the trajectory? If the sample rate is low the speed should be low enought so that position updated will "converge". According to the model of our UAV, the orientation could be got from the Magnetic Flow sensor which is quite fast (up to hundreds of updates per second) additionally integrated angular velocity from gyroscope could be used to further enchance orientation estimation. But what about position? What if there is no markers in view frustum of camera? The second main task is to `filter` the output of the previous phase and try to deduce the pose "for the future". To get the position of the model in the next moment of time, we need to know it's instantenous velocity in the previous moment of time. But we don't have it! From the previous two sampled positions of the camera we can get a rough approximation of the velocity vector. It's not that usefull eiser because the force to the model is applied and the real velocity is changing, but our approximation stays the same for quite long periods of time. That's the point where accelerometer comes to help. The device provides information about the velocity change over time plus the gravitational acceleration. Thus the position at a given time arises as a solution of the Initial value problem of the form:
  S(k) - is known for integer k in range [0, t]
  d^2(S)/dt^2 = acc(t);

it's solution is:
  S(t) = S(k) + integrate_tk^t (vel_k + integrate_tk^t (R^-1 * acc(t)-g_acc) dt);

Where R is a unitary rotation operator in R^3 space that maps world coordinate space into IMU local frame. It is needed since IMU provides data in regard to it's local coordinate space.

## (Part 4) Description of the solution
Model description:

Let's focus on the single model of UAV. Let it be some kind of multirotor, the particular type is not that important as all of them might be represented in a single mathematical model. The UAV model that we try to control has 4 inputs and 2 outputs. The inputs are:
* Image from the camera;
* Magnetic flow sensor - digital compass;
* Angular acceleration sensor - gyroscope;
* Linear acceleration sensor - accelerometer;

The outputs are:
* Torque
* Thrust

To simplify the model the last two output's will be fused together into single one - the force vector which we apply to the model. There are additional limitations applied to the rate how fast this vector changes.

{Image}

Extracting pose from the image:

Pose estimation is of great importance in many computer vision applications: robot navigation, augmented reality, and many more. This process is based on finding correspondences between points in the real environment and their 2d image projection. The problem is called Perspective-n-Point. A commonly used solution to the problem exists for n = 3 called P3P.

When n = 3, the PnP problem is in its minimal form of P3P and can be solved with three point correspondences. However, with just three point correspondences, P3P yields many solutions, so a fourth correspondence is used in practice to remove ambiguity. The setup for the problem is as follows.

Let P be the center of projection for the camera, A, B, and C be 3D world points with corresponding images points u, v, and w. Let X = |PA|, Y = |PB|, Z = |PC|, \alpha = \angle BPC, \beta = \angle APC, \gamma = \angle APB, p = 2\cos\alpha, q = 2\cos\beta, r = 2\cos\gamma, a' = |AB|, b' = |BC|, c' = |AC|. This forms triangles PBC, PAC, and PAB from which we obtain the equation system for P3P:


  \begin{cases}
    Y^2 + Z^2 - YZp - b'^2 &= 0\\
    Z^2 + X^2 - XZq - c'^2 &= 0\\
    X^2 + Y^2 - XYr - a'^2 &= 0\\
  \end{cases}
.
It's common to normalize the image points before solving P3P. Solving the P3P system results in four possible solutions for R and T. The fourth world point D and its corresponding image point z are then used to find the best solution among the four. The algorithm for solving the problem as well as the complete solution classification for it is given in the 2003 IEEE Transactions on Pattern Analysis and Machine Intelligence paper by Gao, et al.[2] An open source implementation of the P3P can be found in OpenCV's calib3d module in the solvePnP function.

This is usually a difficult step, and thus it is common the use of synthetic or fiducial markers to make it easier. One of the most popular approach is the use of binary square fiducial markers. The main benefit of these markers is that a single marker provides enough correspondences (its four corners) to obtain the camera pose. Also, the inner binary codification makes them specially robust, allowing the possibility of applying error detection and correction techniques.

For marker detection and their pose estimation ArUco library was used, which is now a module of industry-standard OpenCV library. An ArUco marker is a synthetic square marker composed by a wide black border and a inner binary matrix which determines its identifier (id). The black border facilitates its fast detection in the image and the binary codification allows its identification and the application of error detection and correction techniques. The marker size determines the size of the internal matrix. For instance a marker size of 4x4 is composed by 16 bits.

{ArUco Marker Image}

Given an image where some ArUco markers are visible, the detection process has to return a list of detected markers. Each detected marker includes:
The position of its four corners in the image (in their original order).
The id of the marker.

The marker detection process is comprised by two main steps:

Detection of marker candidates. In this step the image is analyzed in order to find square shapes that are candidates to be markers. It begins with an adaptive thresholding to segment the markers, then contours are extracted from the thresholded image and those that are not convex or do not approximate to a square shape are discarded. Some extra filtering are also applied (removing too small or too big contours, removing contours too close to each other, etc).
After the candidate detection, it is necessary to determine if they are actually markers by analyzing their inner codification. This step starts by extracting the marker bits of each marker. To do so, first, perspective transformation is applied to obtain the marker in its canonical form. Then, the canonical image is thresholded using Otsu to separate white and black bits. The image is divided in different cells according to the marker size and the border size and the amount of black or white pixels on each cell is counted to determine if it is a white or a black bit. Finally, the bits are analyzed to determine if the marker belongs to the specific dictionary and error correction techniques are employed when necessary.

Data filtering:

Use of just one marker is inplactical since the range of steady detection of single marker is quite low. To get a reliable pose estimation on the whole path of the model averaging of poses is used. For effective averaging there should be a weight function to estimate the quality of the pose for each marker.
A weighted arithmetic mean is used for position deduction, and consecutive spherical linear interpolation for orientation, since it is represented as quaternion.
As a weight function of the visible marker on the camera image the area of the marker in pixels is used. It's quite natural since the further marker from camera is, the smaller is its single pixel and the rougher is the pose estimation. The same happens when the view angle on the marker is acute.

Orientation hold path following:

Once the pose of the UAV is known it shold do something useful like go somewhere or just stay in place. The only control levels the model has are torque and thrust. So the task is to determine the amount of the needed torque and thrust to move somwhere or stay still and hold position. One may numericaly solve the optimisation problem for the shortest time and given limitations. But if one does not need the best time an industry standard for the robotics and engineering area is to use a proportional–integral–derivative controller (PID controller) in such cases. Which is a control loop feedback mechanism. A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error over time by adjustment of a control variable, such as the position of a control valve, a damper, or the power supplied to a heating element, to a new value determined by a weighted sum:

u(t) = K_p e(t) + K_i \int_{0}^{t}e(\tau)d\tau + K_d \frac{de(t)}{dt}
where K_p, K_i, and K_d, all non-negative, denote the coefficients for the proportional, integral, and derivative terms, respectively (sometimes denoted P, I, and D). 

Implementation design details:

As a framework to test the idea a Gazebo robot simulator was used. It provides a good mathematic library, physics simulation, communication, various sensors, debugging tools and plugins in addition to big community.
Publishers and subscribers constitute the core of the transport library (a part of the simulator). It allows to create clean and transparent interfaces between different components.

{ModulesDiagram.png}

Short description of the modules:

App:
WorldFeatures - a set of some features with known locations
ImageLocator - receives an image and locates world features there
Trajectory - defines a trajectory for the UAV to move along
Altitude,XY,Orientaion PIDs - according to trajectory and the pose calculates force and torque needed to follow trajectory.

Simulator:
UAVmodel - Physical model with attached camera and imu sensor, and plugins to publish it's pose and alter force and torque;
Markers - stays in place and publish positions;
Physics - engine simulates physical interactions of the world, handles collisions and forces.
Rendering - visualise what's happening in the world, provides camera sensor input.

Observer - receives real position from the simulator and estimated position from the app and performs data analysis.

Arrows show how the data flows between the modules.

## (Part 5) Result analysis
Result analisys:

Lets take a sample scene {Multiple markers avg}
The scene contains 3 different markers which location is known and a camera.
Image {camera image} shows what does the camera see. Detected markers are enumerated and highlited, then their position is looked-up in a table. Now let's compare the estimated position with the real position of the camera.
The next plot shows the dependency of the length of error vector from the distance from camera to markers:
{length-error.png}
And lets compare it to the single marker tracking.
{length-error-single.png}
Althought it's not significant but one can clearly that on the fist image when all three markers are used to determine position the error starts to grow up a bit later, resulting to the standard deviation of 0.0405 for the first case and 0.0461 for the second one. In this example the frames when marker tracking was lost and the error is larger than 0.3 are dropped.

Inertial prediction testing:

Lets take another scene. The scene contains a quadrotor that tries to follow the curve representad by equation in the polar space r = 5*cos(3*phi).
Image %d shows the general trajectory of the quadrotor. The the simulation provides the position update 10 times per second. One should note that average speed of the quad on this trajectory is 3.45 m/s, so at this rate without the position oracle the model fails to follow the trajectory at all. Image %d displays the close-up view on the part of the path. On the image green solid line displays the sample point - exact position of the model, and purple crosses the predicted trajectory, which is used to update the PID controllers and continue movement along the curve.

## (Part 6) Conclusion

## (Part 7) Sources

## Appendices/Attachments