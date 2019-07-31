#!/usr/bin/env python
# coding: UTF-8

import rospy
from geometry_msgs.msg import Twist
from ultimate_seniorcar.msg import AccidentPredictResult
from ultimate_seniorcar.msg import SeniorcarState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

PUBLISH_RATE = 10.0       # コマンドをパブリッシュする周期

APPROACH_THRESHOLD = 3.0  # 何mまで危険な領域に直進で接近させるか
AVOID_THRESHOLD = 3.5     # 避けるとしたら何mまで余裕がある角度か

NO_INTERVATION_DISTANCE = 3.7    # 足元の情報がないため介入しない原点からの距離

DECELERATION_CONSTANT = 0.5 # 減速時に入力アクセル開度に対し何倍のアクセル開度を出力するか（0~1）

MAX_CHANGE_STEER_ANGLE_BY_STEP = 2.0 # １ステップで変化できる舵角の最大角度。いきなり舵がふられるのを防ぐ
MAX_ACCEL_OPENING = 40
DELAY_TIME = 5

class AvoidPredictedAccident:

	accident_predict_result = AccidentPredictResult()
	seniorcar_odometry = Odometry()
	seniorcar_state   = SeniorcarState()
	seniorcar_command = SeniorcarState()
	seniorcar_command.accel_opening = 0
	seniorcar_state.accel_opening = 0
	seniorcar_command.steer_angle = 0
	seniorcar_command.max_velocity = 2.0
	enable_steer_motor = Bool()
	enable_steer_motor.data = False#for initialize 11/2
	enable_program = Bool()#11/6
	enable_program.data = True#11/6 the seniorcar avoids a step then it turns False
	start_odometry = Odometry()
	odom_recive_flag = True
	update_flag = Bool()
	update_flag.data = True#11/7_23_15
	time_counter = 0  #one way to keep time. 11/7
	time_counter_flag = False
	t1 = rospy.Time()#another way to keep time.11/7
	t2 = rospy.Time()
	t3 = 0
	t1_flag = Bool()
	t1_flag.data = True
	avoid_index1 = 0
	avoid_index2 = 0
	counter = 0
	old_steer_command_angle = 0.0

	def __init__(self):
		rospy.init_node('generate_seniorca_comand_to_avoid_predicted_accident')
		self.pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)
		self.pub2 = rospy.Publisher('enable_motor', Bool, queue_size=10)
		#self.pub3 = rospy.Publisher('enable_brake', Bool, queue_size=10)#for avoid program colision 11/6
	
	def subscribe_predicted_result(self):
		rospy.Subscriber("accident_predict", AccidentPredictResult , self.accidentsubCallback)
		rospy.Subscriber("seniorcar_state", SeniorcarState , self.stateCallback)
		rospy.Subscriber("seniorcar_odometry", Odometry, self.odomCallback)

	def stateCallback(self,msg):
		self.seniorcar_state = msg
		self.seniorcar_command.max_velocity  = msg.max_velocity

	def odomCallback(self,msg):
		if self.odom_recive_flag:
			self.start_odometry = msg
			print "start_odometry"
			print "x:" + str(self.start_odometry.pose.pose.position.x) + "y:" +  str(self.start_odometry.pose.pose.position.y) 
			self.odom_recive_flag = False
		self.seniorcar_odometry = msg

	def accidentsubCallback(self,msg):
		self.accident_predict_result = msg
		self.min_predict_angle = msg.steer_angle[0]
		self.predict_angle_devision = abs( msg.steer_angle[1] - msg.steer_angle[0] )

	def calcSteerAngleToPredictedAngleIndex(self,steer_angle):
		# radに直す
		steer_angle *= 3.14 / 180
		# まだ判定結果を受信していない場合は0を返す
		if len(self.accident_predict_result.steer_angle) < 1:
			return 0
		# 範囲外の処理
		elif steer_angle < self.accident_predict_result.steer_angle[0]:
			return 0
		elif steer_angle > self.accident_predict_result.steer_angle[-1]:
			return int( len(self.accident_predict_result.steer_angle) - 1 )
		else:
			return int( ( steer_angle - self.accident_predict_result.steer_angle[0] ) / ( self.accident_predict_result.steer_angle[1] - self.accident_predict_result.steer_angle[0] ) )

	def update_seniorcar_command(self):

		if self.update_flag.data == False:
			pass

		if self.update_flag.data == True:
			data_num = len(self.accident_predict_result.steer_angle)
			if data_num < 1:
				return 

			current_steer_index = self.calcSteerAngleToPredictedAngleIndex(self.seniorcar_state.steer_angle) # 現在の操舵角度0度の番号
			if current_steer_index == 0:
				current_steer_index += 1
			elif current_steer_index == data_num - 1:
				current_steer_index -= 1

			if self.accident_predict_result.max_distance[current_steer_index] > APPROACH_THRESHOLD and self.accident_predict_result.max_distance[current_steer_index+1] > APPROACH_THRESHOLD and self.accident_predict_result.max_distance[current_steer_index-1] > APPROACH_THRESHOLD:
			# 前方が安全なら操舵角度そのまま
				self.seniorcar_command.steer_angle = self.seniorcar_state.steer_angle
				self.enable_steer_motor.data = False
				self.seniorcar_command.accel_opening = min( self.seniorcar_state.accel_opening , MAX_ACCEL_OPENING )
			else:
			# 安全に走行できる経路の中で現在の角度に近い場所を探す。そもそも無ければ停止する
				avoid_index = -1
				for i in range(0,data_num-1):
					if self.accident_predict_result.max_distance[i] > AVOID_THRESHOLD and self.accident_predict_result.max_distance[i+1] > APPROACH_THRESHOLD and self.accident_predict_result.max_distance[i-1] > APPROACH_THRESHOLD:
						if abs( current_steer_index - i ) < abs( current_steer_index - avoid_index ):
							avoid_index = i
				if avoid_index == -1:
					self.seniorcar_command.accel_opening = min( self.seniorcar_state.accel_opening * DECELERATION_CONSTANT , MAX_ACCEL_OPENING * DECELERATION_CONSTANT)
					self.enable_steer_motor.data = False
				else:
					self.seniorcar_command.accel_opening = min( self.seniorcar_state.accel_opening , MAX_ACCEL_OPENING )
					self.enable_steer_motor.data = True
					if avoid_index < 5 and avoid_index > 2:#to adjust offset.11/6 
						self.avoid_index1 = avoid_index-2
						self.seniorcar_command.steer_angle = self.accident_predict_result.steer_angle[self.avoid_index1]  * 180.0 /3.14
						if self.t1_flag.data == True:
							self.t1 = rospy.Time.now()
							self.t1_flag.data = False
				#elif avoid_index == 7: 
					#self.seniorcar_command.steer_angle = self.accident_predict_result.steer_angle[avoid_index]  * 180.0 /3.14
					elif avoid_index > 9 and avoid_index < 12: 
						self.avoid_index1 = avoid_index+2
						self.seniorcar_command.steer_angle = self.accident_predict_result.steer_angle[self.avoid_index1]  * 180.0 /3.14
						if self.t1_flag.data == True:
							self.t1 = rospy.Time.now()
							self.t1_flag.data = False
					else:
						self.avoid_index1 = avoid_index
						self.seniorcar_command.steer_angle = self.accident_predict_result.steer_angle[self.avoid_index1]  * 180.0 /3.14#11/6

				# 急激に変化しないようにする処理
					if abs( self.seniorcar_command.steer_angle - self.old_steer_command_angle ) > MAX_CHANGE_STEER_ANGLE_BY_STEP:
						if self.seniorcar_command.steer_angle > self.old_steer_command_angle:
							self.seniorcar_command.steer_angle = self.old_steer_command_angle + MAX_CHANGE_STEER_ANGLE_BY_STEP
						elif  self.seniorcar_command.steer_angle < self.old_steer_command_angle:
							self.seniorcar_command.steer_angle = self.old_steer_command_angle - MAX_CHANGE_STEER_ANGLE_BY_STEP

			self.old_steer_command_angle = self.seniorcar_command.steer_angle

	def func(self):
		pass

	def calculate_and_publish_command(self):
		rate = rospy.Rate(PUBLISH_RATE)
		while not rospy.is_shutdown():
			# 最初は足元の情報がないので介入なし
			if pow( self.seniorcar_odometry.pose.pose.position.x - self.start_odometry.pose.pose.position.x , 2 ) +  pow( self.seniorcar_odometry.pose.pose.position.y - self.start_odometry.pose.pose.position.y , 2 )  < pow( NO_INTERVATION_DISTANCE , 2 ):
				self.enable_steer_motor.data = False
				self.seniorcar_command.accel_opening = min( self.seniorcar_state.accel_opening , MAX_ACCEL_OPENING )
				self.seniorcar_command.steer_angle = self.seniorcar_state.steer_angle

			else:
				self.update_seniorcar_command()#11/7_23_15
				if self.t1_flag.data == False:
					self.t2 = rospy.Time.now()
					self.t3 = self.t2.secs-self.t1.secs
					print(self.t3) #to check t3 11/8_10_25
					 #11/8_9_20 I checked once t1_flag turn False . and then turn off coment out
					 #11/7_23_15
					if self.t3 < DELAY_TIME and self.t3 > 0 or self.t3 == DELAY_TIME : # I will check here .11/7_9_28
						self.update_seniorcar_command()
						print("Hello")
						if self.t1_flag.data == True:
							print ("True")
						#self.func()
					elif self.t3 > DELAY_TIME and self.t3 < DELAY_TIME*2:
						self.update_flag.data = False
						#self.seniorcar_command.accel_opening =  0
						self.avoid_index2 = 14 - self.avoid_index1
						self.seniorcar_command.steer_angle = self.accident_predict_result.steer_angle[self.avoid_index2]
						print("avoid_index1")
						print(self.avoid_index1)
						print("avoid_index2")
						print(self.avoid_index2)
						#self.seniorcar_command.accel_opening = 0
					elif self.t3 == 0 or self.t3 < 0:
						self.func()
						print("start")
					else:
						print("I'll update")
						self.t1_flag.data = True
						self.update_flag.data = True#11/7_23_15
			self.pub.publish(self.seniorcar_command)
			self.pub2.publish(self.enable_steer_motor)
			rate.sleep()

if __name__ == '__main__':
	calclater = AvoidPredictedAccident()
	calclater.subscribe_predicted_result()
	calclater.calculate_and_publish_command()
