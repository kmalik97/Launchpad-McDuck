# Launchpad-McDuck

Adding a note on how to startup ssh duckiebot:
1. git pull to get the most recent changes

2. on computer:
	check ip address, goto setup.bash to make ip addr is correct
	source setup.bash
	roscore
	run image_processing on computer

3. ssh into duckiebot: ssh duckiebot@duckiebot01.local	
	check ip addresses of setup.bash
	source setup.bash
	run camera_interface, motion_logic, and motion_interface all on duckiebot	
