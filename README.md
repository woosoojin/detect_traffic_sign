# detect_traffic_sign
<h2> path: turtlebot3_autorace/turtlebot3_autorace_detect</h2>

<h1>1. How to use</h1>

<pre><code>[remote PC]<br>
$ roscore
</code></pre><br>

To activate all sensors:
<pre><code>[TurtleBot SBC] <br>
$ roslaunch turtlebot3_bringup alltogether.launch
</code></pre>

Settings for recognition
<pre><code><br>[remote PC]<br>
$ export AUTO_IN_CALIB=action
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
$ export AUTO_EX_CALIB=action
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_extrinsic_camera_calibration.launch
</code></pre>

<br>
To detect various traffic sign
<pre><code>[remote PC]<br>
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_sign.launch
</code></pre> alternative: $ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_sign_image.launch (utilize /detect/image_traffic_sign/compressed)

<br><br>
To decide speed depends on different traffic sign
<pre><code>$ rosrun turtlebot3_autorace_detect decider
</code></pre>

<br>
To send velocity information to turtlebot3 wiffle pi
<pre><code>$ rosrun turtlebot3_autorace_detect detect_talker
</code></pre>
