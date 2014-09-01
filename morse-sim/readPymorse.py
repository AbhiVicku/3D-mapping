#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  readPymorse.py
#  This program reads data published over socket using pymorse.
#  Copyright 2014 abhinav <abhinav@lab-pc>
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
from pymorse import Morse
import base64

def printer(data):
	print("data * " + str(data))

def main():
	print("1")
	with Morse() as morse:
		print("3")
		msg = morse.robot.camera.get()
		data = base64.b64decode(msg['points'])
		for i in range(0, len(data) - 12, 12):
			xyz = struct.unpack('fff', data[i:i+12])
			print(xyz)
		#print(data)
		morse.sleep(0.2)
	return 0

if __name__ == '__main__':
	main()

