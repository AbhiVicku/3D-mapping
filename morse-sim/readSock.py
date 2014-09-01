#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  readSock.py
#  This program reads the data published over socket. 
#  It is used with Morse where the data is publishd over socket
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

import sys, socket, json

def main():
	host = "localhost"
	port = 60001
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((host, port))
	morse = sock.makefile("r")

	data = json.loads(morse.readline())

	print type(data)
	return 0

if __name__ == '__main__':
	main()

