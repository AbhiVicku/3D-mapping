#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  simDepth.py
#  
#	This program simulates environment with robot publishing depth
#	data over socket.
#  Copyright 2014 tom-lab <abhinav@tomlab>
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
import sys
import struct
import base64
from morse.builder import *



if __name__=="__main__":
		
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

		robot.append(camera)
		camera.add_stream('socket')

		env = Environment('indoors-1/boxes')

		

