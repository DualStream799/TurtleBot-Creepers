#! /usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "DualStream799"




import rospy
import numpy
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
import cv2
import sys
# Importing custom libraries:
from ROS_OpenCV_Pythonlib.bot_module import ControlBotModule, VisionBotModule, SupportBotModule

x = 0
y = 0
z = 0 
id = 0
image = None
marcadores = []
angulo_marcador_robo = 0


export_frame = None
track_contour_point = None
screen_point = None
# Hue value for masks:
yellow_hue = 60
green_hue = 126
blue_hue = 213
pruple_hue = 293
bot = ControlBotModule()
visor = VisionBotModule()

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	global angulo_marcador_robo
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		#print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		if id == 2:
			# Separa as translacoes das rotacoes
			x = trans.transform.translation.x
			y = trans.transform.translation.y
			z = trans.transform.translation.z
			# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
			# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
			# no eixo X do robo (que e'  a direcao para a frente)
			t = transformations.translation_matrix([x, y, z])
			# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
			r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
			m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
			z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
			v2 = numpy.dot(m, z_marker)
			v2_n = v2[0:-1] # Descartamos a ultima posicao
			n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
			x_robo = [1,0,0]
			cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
			angulo_marcador_robo = math.degrees(math.acos(cosa))

			# Terminamos
			print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))


def on_frame(image):
	global export_frame
	global track_contour_point
	global screen_point
	# Converts image to proper format:
	frame = bot.convert_compressed_to_cv2(image)
	# Resizes the frame to fit on the screen:
	frame = cv2.resize(frame, (frame.shape[1]/2, frame.shape[0]/2))
	screen_point = frame.shape
	# Converts frame to all spacecolors:
	bgr_frame, gray_frame, rgb_frame, hsv_frame = visor.frame_spacecolors(frame)
	#yellow_rgb = (239, 239, 0)
	# Creates masks based on the color:
	yellow_mask = visor.frame_mask_hsv(hsv_frame, yellow_hue, 10, value_range=(180, 255))
	green_mask = visor.frame_mask_hsv(hsv_frame, green_hue, 10, [80, 255], [50, 255])
	blue_mask = visor.frame_mask_hsv(hsv_frame, blue_hue, 10, [80, 255], [50, 255])
	purple_mask = visor.frame_mask_hsv(hsv_frame, green_hue, 10, [80, 255], [50, 255])
	# Reduces noise:
	yellow_mask_clean = visor.morphological_transformation(yellow_mask, 'opening', 4)
	green_mask_clean = visor.morphological_transformation(green_mask, 'opening', 4)
	blue_mask_clean = visor.morphological_transformation(blue_mask, 'opening', 4)
	purple_mask_clean = visor.morphological_transformation(purple_mask, 'opening', 4)
	# Detects yellow contours:
	contours, tree = visor.contour_detection(yellow_mask)
	# Finds the biggest contour:
	biggest_contour = visor.contour_biggest_area(contours)

	if biggest_contour is not None:
		# Draws the contour:
		visor.contour_draw(bgr_frame, biggest_contour, color=[0,0, 255])
		# Draws a aim on the center of the biggest contour:
		track_contour_point = visor.contour_features(biggest_contour, 'center')
		visor.draw_aim(bgr_frame, track_contour_point, color=[255,0,0])
	# Display current frame
	export_frame = bgr_frame

if __name__=="__main__":
	print("Coordenadas configuradas para usar robô virtual, para usar webcam USB altere no código fonte a variável frame")

	rospy.init_node("marcador") # Como nosso programa declara  seu nome para o sistema ROS

	#recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) # Para podermos controlar o robo
	robo_camera = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, on_frame, queue_size=4, buff_size = 2**24)

	tfl = tf2_ros.TransformListener(tf_buffer) # Para fazer conversao de sistemas de coordenadas - usado para calcular angulo

	try:
		# Loop principal - todo programa ROS deve ter um
		while not rospy.is_shutdown():
			if track_contour_point is not None and screen_point is not None:
				if (track_contour_point[0] >= screen_point[0]):
					bot.angular_z = -0.1
				else:
					bot.angular_z = 0.1

				bot.linear_x = 0.1
			else:
				bot.angular_z = 0.1
				bot.linear_x = 0
			velocidade_saida.publish(bot.main_twist())
			#rospy.sleep(0.5)
			if export_frame is not None:
				visor.display_frame('frame', export_frame)

			# Waits for a certain time (in milisseconds) for a key input ('0xFF' is used to handle input changes caused by NumLock):
			delay_ms = 60
			key_input = cv2.waitKey(delay_ms) & 0xFF
			# Exit the program:
			if  key_input == ord('q'):
				bot.linear_x = 0
				bot.angular_z = 0
				velocidade_saida.publish(bot.main_twist())
				break
	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")
		#visor.display_terminate()


