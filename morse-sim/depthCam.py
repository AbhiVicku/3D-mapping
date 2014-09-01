import sys
import math
import struct
import base64
from morse.testing.testing import MorseTestCase
from pymorse import Morse

try:
	from morse.builder import *
except ImportError:
	pass

class DepthCameratest(MorseTestCase):
	def setUpEnv(self):
		robot = ATRV()
		motion = MotionVW()
		robot.append(motion)
		motion.add_stream('socket')

		camera = DepthCamera()
		camera.translate(z=1)
		camera.frequency(3)

		robot.append(camera)
		camera.add_stream('socket')

		env = Environment('indoors-1/boxes')

	def test_depth_camera(self):
		with Morse() as morse:
			morse.robot.publish({'v':1,'w',1})

			for step in range(5):
				msg = morse.robot.camera.get()
				data = base64.b64decode(msg['points'])

				for i in range(0,len(data)-12,12):
					xyz = struct.unpack('fff',data[i:i+12])
					self.assertTrue(xyz[2]>=1 and xyz[2]<=20)
				morse.sleep(0.2)

if __name__==__'main'__:
        from morse.testing.testing import main
        main(DepthCameraTest)

