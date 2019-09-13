#!/usr/bin/env python
#encoding: utf8
import rospy, cv2, math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class Deep():
	def __init__(self):
		sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
		self.pub = rospy.Publisher("face", Image, queue_size=1)
		self.bridge = CvBridge()
		self.image_org = None
		self.updn = 0.0
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rospy.wait_for_service('/motor_on')
		rospy.wait_for_service('/motor_off')
		rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
		rospy.ServiceProxy('/motor_on', Trigger).call()

	def red_detect(self, img):
		# HSV色空間に変換
    		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		# 赤色のHSVの領域1
		hsv_min = np.array([0,150,150])
		hsv_max = np.array([int(15*255/360),255,255])
		mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

		# 赤色のHSVの領域2
		hsv_min = np.array([150,127,0])
		hsv_max = np.array([255,255,255])

		mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

		return mask1 + mask2

	def monitor(self,rect,org):
		if rect is not None:
			cv2.rectangle(org,tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]),(0,255,255),4)

		# 真ん中に十字を描く
		height = org.shape[0]
		width = org.shape[1]
		cv2.line(org, (int(width/2-width/30), int(height/2)), (int(width/2+width/30), int(height/2)), (255,255,255), 10)
		cv2.line(org, (int(width/2), int(height/2-height/30)), (int(width/2), int(height/2+height/30)), (255,255,255), 10)

		# 方向（テキスト）を描く
		if self.updn == 0.0:
			direction = "good"
		elif self.updn < 0.0:
			direction = "up"
		else:
			direction = "down"
		fontType = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(org, direction, (0, int(height-height/8)), fontType, 3, (255, 255, 255), 3)

		self.pub.publish(self.bridge.cv2_to_imgmsg(org, "bgr8"))

	def get_image(self,img):
		try:
			self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

	def detect_point(self):
		if self.image_org is None:
			return None
		org = self.image_org
		mask = self.red_detect(org)

		# 作成したmaskの中で、一番大きいものだけを追跡する（矩形で囲む）
		image, contours, hierarchy  = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		rects = []
		for contour in contours:
			approx = cv2.convexHull(contour)
			rect = cv2.boundingRect(approx)
			rects.append(np.array(rect))

		if len(rects) == 0:
			self.monitor(None,org)
			return None
		if len(rects) > 0:
			r = max(rects, key=(lambda x: x[2] * x[3]))

		self.monitor(r,org)
		return r

	def updown_vel(self):
		r = self.detect_point()
		if r is None:
			return 0.0
		r_center = r[0] + r[2]/2 #検出した矩形の真ん中（x軸方向）
		icw = self.image_org.shape[1]/2 #画像の真ん中（x軸方向）
		mgn = icw/50 #ズレの許容範囲
		lower = icw - mgn
		higher = icw + mgn
		if(r_center >= lower and r_center <= higher):
			self.updn = 0.0 #'good'
		elif(r_center < lower):
			self.updn = -0.01 # 'up'
		else:
			self.updn = 0.01 #'down'

	def control(self):
		m = Twist()
		m.angular.z = 0.0
		self.updown_vel()
		m.linear.x = self.updn
		self.cmd_vel.publish(m)

if __name__ == '__main__':
	rospy.init_node('deep_control')
	d = Deep()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		d.control()
		rate.sleep()


