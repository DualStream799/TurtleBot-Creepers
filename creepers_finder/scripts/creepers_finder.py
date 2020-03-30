#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = "DualStream799"

# Importing Libraries

from ROS_OpenCV_Pythonlib.bot_module import TurtleBot


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

bot = TurtleBot()
# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

cap_lock = 50

frame_capturado = None

def run_all_frames(imagem):

	global cap_lock
	global frame_capturado

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()


		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		# cv_image = cv2.flip(cv_image, -1)
		#media, centro, maior_area =  cormodule.identifica_cor(cv_image)
		#bot.frame_capture('creepers_gazebo', cv_image)

		if cap_lock == 0:
			frame_capturado = cv_image
		else:
			cap_lock -= 1


		depois = time.clock()

		return dado


	except CvBridgeError as e:
		print('ex', e)

if __name__=="__main__":
	rospy.init_node("cor")

	# topico_imagem = "/kamera"
	topico_imagem = "/camera/rgb/image_raw/compressed"
	
	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, run_all_frames, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	
	try:

		while not rospy.is_shutdown():
			vel = Twist(bot.vector_zero, bot.vector_zero)
			bot.frame_capture('creepers_gazebo', frame_capturado)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")