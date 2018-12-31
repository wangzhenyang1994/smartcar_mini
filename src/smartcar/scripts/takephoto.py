import cv2
import time
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist



i = 0
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

cmd_vel=Twist()

rospy.init_node('takephoto',anonymous=True)
pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
rate=rospy.Rate(20)

vel=0.1

while(1):
    # get a frame
    ret, frame = cap.read()
    # show a frame
    cv2.imshow("capture", frame)
    cv2.imwrite(str(i).zfill(5)+".jpg", frame)
    i+=1
    cmd_vel.linear.x=vel
    cmd_vel.angular.z=0
    if(i % 100==0):
        vel=vel*(-1)
    if(i==200):
        cmd_vel.linear.x=0
        cmd_vel.angular.z=0
        pub.publish(cmd_vel)
        break
    pub.publish(cmd_vel)
    rate.sleep()

        
    
cap.release()
cv2.destroyAllWindows()

