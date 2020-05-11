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
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
import cv2
import sys
import time

# Importing custom libraries:
from ROS_OpenCV_Pythonlib.bot_module import ControlBotModule, VisionBotModule, SupportBotModule

x = 0
y = 0
z = 0 
id = 0
image = None
marcadores = []
angulo_marcador_robo = 0

status = 'run'
diff_pos = []
diff_ori = []
export_frame = None
track_contour_point = None
screen_point = None
# Hue value for masks:
yellow_hue = 60
green_hue = 123
blue_hue = 207
pink_hue = 309

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


def check_frame_delay(image):
	atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

	now = rospy.get_rostime()
	imgtime = image.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()

		on_frame(image)

		depois = time.clock()

		return 
	except CvBridgeError as e:
		#print('ex', e)
		pass

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
	# Creates masks based on the color:
	yellow_mask = visor.frame_mask_hsv(hsv_frame, yellow_hue, 10, value_range=(180, 255))
	green_mask = visor.frame_mask_hsv(hsv_frame, green_hue, 10, (80, 255),(50, 255))
	blue_mask = visor.frame_mask_hsv(hsv_frame, blue_hue, 10, (80, 255), (50, 255))
	pink_mask = visor.frame_mask_hsv(hsv_frame, pink_hue, 10, (80, 255), (50, 255))
	# Gets rid of noise:
	yellow_mask_clean = visor.morphological_transformation(yellow_mask, 'opening', 4)
	green_mask_clean = visor.morphological_transformation(green_mask, 'opening', 4)
	blue_mask_clean = visor.morphological_transformation(blue_mask, 'opening', 4)
	pink_mask_clean = visor.morphological_transformation(pink_mask, 'opening', 4)
	# Detects contours for each color:
	yellow_contours, tree = visor.contour_detection(yellow_mask_clean)
	green_contours, tree = visor.contour_detection(green_mask_clean)
	blue_contours, tree = visor.contour_detection(blue_mask_clean)
	pink_contours, tree = visor.contour_detection(pink_mask_clean)

	# Finds the closes creeper selecting the biggest contour between all 3 masks (green, blue and pink):
	biggest_contours = []
	if len(green_contours) != 0:
		# Finds the biggest green contour:
		green_biggest_contour = visor.contour_biggest_area(green_contours)
		# Draws the contour:
		biggest_contours.append(green_biggest_contour)
		x, y, w, h = visor.contour_features(green_biggest_contour, 'str-rect')
		green_text_point = visor.convert_dimensions_to_points((x, y-15, w, h))[0]
		visor.draw_rectangle(bgr_frame, (x, y, w, h), color=(13, 253, 0))
		visor.draw_text(bgr_frame, "Green", green_text_point, thickness=1, font_size=0.5, text_color=(13, 253, 0))

	if len(blue_contours) != 0:
		# Finds the biggest blue contour:
		blue_biggest_contour = visor.contour_biggest_area(blue_contours)
		# Draws the contour:
		biggest_contours.append(blue_biggest_contour)
		x, y, w, h = visor.contour_features(blue_biggest_contour, 'str-rect')
		blue_text_point = visor.convert_dimensions_to_points((x, y-15, w, h))[0]
		visor.draw_rectangle(bgr_frame, (x, y, w, h), color=(255, 145, 17))
		visor.draw_text(bgr_frame, "Blue", blue_text_point, thickness=1, font_size=0.5, text_color=(255, 145, 17))

		
	if len(pink_contours) != 0:
		# Finds the biggest pink contour:
		pink_biggest_contour = visor.contour_biggest_area(pink_contours)
		# Draws the contour:
		biggest_contours.append(pink_biggest_contour)
		x, y, w, h = visor.contour_features(pink_biggest_contour, 'str-rect') 
		pink_text_point = visor.convert_dimensions_to_points((x, y-15, w, h))[0]
		visor.draw_rectangle(bgr_frame, (x, y, w, h), color=(216, 0, 255))
		visor.draw_text(bgr_frame, "Pink", pink_text_point, thickness=1, font_size=0.5, text_color=(216, 0, 255))

	# Draws the yellow contour and the center of it:
	if len(yellow_contours) != 0:
		# Finds the biggest contour:
		yellow_biggest_contour = visor.contour_biggest_area(yellow_contours)
		# Draws the contour:
		visor.contour_draw(bgr_frame, yellow_biggest_contour, color=(0,0,0))
		# Draws a aim on the center of the biggest contour:
		track_contour_point = visor.contour_features(yellow_biggest_contour, 'center')

		visor.draw_aim(rgb_frame, track_contour_point)

	closest_creeper = []
	if len(biggest_contours) > 1:
		closest_creeper.append(visor.contour_biggest_area(biggest_contours))
	elif len(biggest_contours) == 1:
		closest_creeper.append(biggest_contours[0])


	# Draws an rectangle around the closest creeper detected:
	if len(closest_creeper) == 1:
		x, y, w, h = visor.contour_features(closest_creeper[0], 'str-rect')
		closest_text_point = visor.convert_dimensions_to_points((x-5, y-25, w, h))[0]
		visor.draw_text(bgr_frame, "Closest", closest_text_point, thickness=1, font_size=0.5, text_color=(245, 242, 66))

	# Display current frame:
	export_frame = bgr_frame

def odometry_position(msg):
	global status

	pose = msg.pose.pose
	bot.odom_scan(msg)
	if status == 'odom_save':
		bot.set_goal(bot.odom_x, bot.odom_y)
		print('Odometry saved')
		status = 'run'
	elif status == 'comeback':
		bot.update_goal_state()

first_time = True
aligned = False
if __name__=="__main__":

	rospy.init_node("marcador")
	rate = rospy.Rate(5)

	#recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
	velocidade_saida = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Para podermos controlar o robo
	robo_camera = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, on_frame, queue_size=4, buff_size=2**24)
	obodmetry_sub = rospy.Subscriber('/odom', Odometry, odometry_position)

	tfl = tf2_ros.TransformListener(tf_buffer) # Para fazer conversao de sistemas de coordenadas - usado para calcular angulo

	try:
		 #Loop principal - todo programa ROS deve ter um
		while not rospy.is_shutdown():
			# 'run' status keeps the robot on track:
			if status == 'run' and track_contour_point is not None and screen_point is not None and export_frame is not None:
				if (track_contour_point[0] >= screen_point[0]):
					bot.angular_z = -0.1
				else:
					bot.angular_z = 0.1
				bot.linear_x = 0.1
			# 'comeback' status makes the robot align with the initial point:
			elif status == 'comeback' and abs(bot.goal_angle - bot.odom_yaw) > 0.1:
					bot.linear_x = 0.0
					bot.angular_z = 0.1
					print('Aligning with goal: {}'.format(abs(bot.goal_angle - bot.odom_yaw)))

			elif status == 'comeback' and abs(bot.goal_angle - bot.odom_yaw) <= 0.1:
				velocidade_saida.publish(bot.stop_twist())
				rospy.sleep(3)
				print("Aligned")
				status = 'aligned'				
			# 'aligned' status makes the robot come back to the marked point:
			elif status == 'aligned' and abs(bot.goal_angle - bot.odom_yaw) > 0.1:
				status = 'comeback'
			elif status == 'aligned' and bot.goal_distance > 0.1 and (bot.goal_angle - bot.odom_yaw) > 0.1:
				bot.linear_x = -0.1
				bot.angular_z = 0.1
				print("Correting trajectory")
			elif status == 'aligned' and bot.goal_distance > 0.1 and (bot.goal_angle - bot.odom_yaw) < -0.1:
				bot.linear_x = -0.1
				bot.angular_z = -0.1
				print("Correcting trajectory 2")
			elif status == 'aligned' and bot.goal_distance > 0.1 and (bot.goal_angle - bot.odom_yaw) < 0.1 and (bot.goal_angle - bot.odom_yaw) > -0.1:
				bot.linear_x = -0.1
				bot.angular_z = 0.0
				print("Trajectory correct")		
			elif status == 'aligned' and bot.goal_distance <= 0.1:
				# implement a alignment from the previous state
				bot.linear_x = 0.0
				bot.angular_z = 0.0
				print("Goal reached")
				status = 'run'

			# Any other status:
			else:
				bot.angular_z = 0.1
				bot.linear_x = 0
			velocidade_saida.publish(bot.main_twist())
			rate.sleep()

			if export_frame is not None:
				visor.display_frame('frame', export_frame)

			# Waits for a certain time (in milisseconds) for a key input ('0xFF' is used to handle input changes caused by NumLock):
			delay_ms = 60
			key_input = cv2.waitKey(delay_ms) & 0xFF
			# Exit the program:
			if  key_input == ord('q'):
				velocidade_saida.publish(bot.stop_twist())
				break
			if key_input == ord('r'):
				status = 'comeback'
			if key_input == ord('s'):
				status = 'odom_save'

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")
		velocidade_saida.publish(bot.stop_twist())
