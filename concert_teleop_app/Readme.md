## Demo Usage

### Launching

```
> rocon_launch turtle_concert teleop.concert --screen
```

### Getting Available Teleops

```
> rostopic echo /services/teleop/available_teleops
```

### Capturing a Teleop Turtle

Use the service pair `rocon_service_msgs/CaptureTeleop.pair` in

```
/services/teleop/capture_teleop/Request
/services/teleop/capture_teleop/Response
```

Example of a service pair client is in [rocon_python_comms/test](https://github.com/robotics-in-concert/rocon_tools/blob/hydro-devel/rocon_python_comms/tests/service_pair_timeouts/test_service_pair_timeouts.py)

You probably want to use a callback, show a loading thingy, when the callback fires, kill the loading thingy.

If you get success, the robot turtle_concert/teleop rapp will start.

### Using a Teleop Turtle

Once started you should see topics like:

```
/kobukia35e168e1235632/cmd_vel
/kobukia35e168e1235632/compressed_image
```

### Problems

* Q) How do you know what kobukia35e168e1235632 is?
* A) Dan should supply this in the capture_teleop service pair response (along with result = true).
 

