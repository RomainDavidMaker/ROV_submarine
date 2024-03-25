

todo : 
- send to the serial a round value to reduce message size
- need to check the frequency to send the sensor values and read motors 
- make a better ui 
- try a ui with rqt rqt plot
- change sensor type use dedicated type instead than a list? no i cant publish mutliple type of message in a single topic
- create a package ?
- implement a stabilisation using the quaternion 
- add a fan control
- average reading on the esp
- add a low bat led state flashing
- safety 0 twist message if closing control_ps4


First test in the pool (25/03/24):
- need a startup file to launch multiple terminals + start ssh
- M3, M7, x forward, z rotation and others? are reversed
- loose connection with controller => ok
- need to add a safety if the controller is disconnected (send 0 command and try to reconnect it) => ok
- display controller battery and warning if disconnected => unknow message
- need more power, increase max esc command => ok
- reduce rate on z rotation

Second test in the pool:
- change  y translation with z rotation (easier to pilot)
- check buoyancy need to add foam
- check motors direction
- add to the control node a safety to set a 0 twist if no command received
