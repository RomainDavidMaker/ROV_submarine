

todo : 
- send to the serial a round value to reduce message size /ok
- need to check the frequency to send the sensor values and read motors 
- make a better ui 
- try a ui with rqt rqt plot
- create a package
- implement a stabilisation using the quaternion /ok
- add a fan control /ok
- average reading on the esp 
- add a low bat vibrating the remote?
- safety 0 twist message if closing control_ps4 /ok
- redo the screen message: add a connection loss
- remake grip part for the tube
- create a part to add weight at the bottom of the sub, density sea is 1.025, weight of the sub is around 6kg need to add 150g
- add a button switch to switch mode



First test in the pool (25/03/24):
- need a startup file to launch multiple terminals + start ssh
- M3, M7, x forward, z rotation and others? are reversed
- loose connection with controller /ok
- need to add a safety if the controller is disconnected (send 0 command and try to reconnect it) /ok
- display controller battery and warning if disconnected => unknow message
- need more power, increase max esc command /ok
- reduce rate on z rotation

Second test in the pool:
- change  y translation with z rotation (easier to pilot) /ok
- check buoyancy need to add foam /ok
- check motors direction /ok
- add to the control node a safety to set a 0 twist if no command received /ok

Test3:
- added a bottle for buoyancy, almost 1L, added on the top of the sub, gived a lot of stabilisation
- tried horinzon stab, PID around (.5, 0, .1), instable at some point, the bottle does most of the stabilisation
- possible to rely mostly on the buoyancy for the stabilisation

Test 4:
- added foam for buoyancy. the sub us less stable than with the bottle and it tilts backward
- solution for tilting: shift the tube front or back to shift the center of gravity?
- to give more stability: add a foam handle? add weight on the bottom?
- the horizon stability is unstable and maybe not needed with the stabulity by the buoyancy

Test 5:
- added a max pid output
- need to try to set pid values one axis at a time



