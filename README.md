# detect_traffic_sign

<h1>1. How to use</h1>

<pre><code>[remote PC]<br>
$ roscore
</code></pre><br>

To activate all sensors:
<pre><code>[TurtleBot SBC] <br>
$ roslaunch turtlebot3_bringup alltogether.launch
</code></pre>


<br>
To detect various traffic sign
<pre><code>[remote PC]<br>
$ rosrun turtlebot3_autorace_detect detect_sign
</code></pre> alternative: $ rosrun turtlebot3_autorace_detect detect_sign_image (utilize /detect/image_traffic_sign/compressed)

<br><br>
To decide speed depends on different traffic sign
<pre><code>$ rosrun turtlebot3_autorace_detect decider
</code></pre>

<br>
To send velocity information to turtlebot3 wiffle pi
<pre><code>$ rosrun turtlebot3_autorace_detect detect_talker
</code></pre>
