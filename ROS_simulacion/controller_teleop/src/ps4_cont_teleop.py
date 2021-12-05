import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyPS4Controller.controller import Controller

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_up_arrow_press(self):
       return 'up'

    def on_down_arrow_press(self):
       return 'down'

    def on_left_arrow_press(self):
       return 'left'

    def on_right_arrow_press(self):
       return 'right'

class Controller_Teleop:
    def __init__(self):
        rospy.init_node('ps4_controller')
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.imshow('image', cv_image)
        except CvBridgeError as e:
            print(e)

        controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
        controller.listen(timeout=60)

        
if __name__ == '__main__':

    ct = Controller_Teleop()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)