#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  hri.py
#  
#  Copyright 2014 lab <lab@lab-pc>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

from morse.builder import *

def main():
	human = Human()
	human.use_world_camera()
	#human.disable_keyboard_control()
	pose = Pose()
	human.append(pose)
	pose.add_stream('socket')
	
	robot = ATRV()
	motion = MotionVW()
	robot.append(motion)
	motion.add_stream('socket')

	robot.translate(1.0, 0.0, 0.0)

	keyboard = Keyboard()
	robot.append(keyboard)
	keyboard.properties(ControlType = 'Position')
	
	
		
	camera = DepthCamera()
	camera.translate(z=1)
	camera.frequency(3)
	camera.properties(cam_width=640,cam_height=480,retrieve_zbuffer=True,cam_far=4.0)
		

	robot.append(camera)
	camera.add_stream('socket')
	camera.add_service('socket')
	env = Environment('apartment')
	env.show_framerate(True)
	
	
	return 0

if __name__ == '__main__':
	main()

