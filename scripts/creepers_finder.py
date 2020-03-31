#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = "DualStream799"

# Importing custom libraries:
from bot_module import ControlBotModule, VisionBotModule
import cormodule
# Importing ROS related libraries:
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rospy
# Importing support related libraries:
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import math
import cv2
import time

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

bot = ControlBotModule()
visor = VisionBotModule()
# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

cap_lock = 50

frame_capturado = None

def run_all_frames(imagem):
	global cv_image
	global media
	global centro
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
		#cv_image = cv2.flip(cv_image, -1)
		media, centro, maior_area =  cormodule.identifica_cor(cv_image)

		if cap_lock == 0:
			frame_capturado = cv_image
		else:
			cap_lock -= 1


		depois = time.clock()

		return 


	except CvBridgeError as e:
		print('ex', e)

if __name__=="__main__":
	rospy.init_node("cor")

	# topico_imagem = "/kamera"
	topico_imagem = "/camera/rgb/image_raw/compressed"
	print("Usando ", topico_imagem)
	
	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, run_all_frames, queue_size=4, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, bot.laser_scan)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	bot.linear_x = 0.1
	try:

		while not rospy.is_shutdown():
			#bot.frame_capture('creepers_gazebo', frame_capturado)
			
			if len(media) != 0 and len(centro) != 0:
				#print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
				#print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))
				if bot.ahead_last >= 0.60:
					if (media[0] > centro[0]):
						bot.angular_z = -0.1
					elif (media[0] < centro[0]):
						bot.angular_z = 0.1
				else:
					bot.linear_x = 0
					bot.angular_z = 0

			velocidade_saida.publish(bot.main_twist())
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")