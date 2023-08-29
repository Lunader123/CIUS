# -*-coding:utf-8 -*-
# 1.导包
import rospy, sys
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped
from std_msgs.msg import Float64
import math

if __name__ == "__main__":

    # 2.初始化 ROS 节点
    rospy.init_node("mytf_listener")

    # 相机关节角度发布器 
    joint1pub = rospy.Publisher('/jaka/joint1_position_controller/command', Float64, queue_size=1000)
    joint2pub = rospy.Publisher('/jaka/joint2_position_controller/command', Float64, queue_size=1000)
    angle1=Float64()
    angle2=Float64()
    
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        try:
        # 4.求出link6在cam_base_link里面的位置
            #lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            tfs = buffer.lookup_transform("cam_base_link","link_6",rospy.Time(0))
            x = tfs.transform.translation.x
            y = tfs.transform.translation.y
            z = tfs.transform.translation.z
            sys.stdout.write('\r相对位置：x:{0:.3f}  y:{1:.3f}  z:{2:.3f}'.format(x,y,z))
            # 求解旋转角：
            joint1_angle = math.atan2(y, x)
            angle1.data = joint1_angle
            joint1pub.publish(angle1)
            # 求解俯仰角
            l = math.sqrt(x*x + y*y)
            z2 = z - 0.077 #我粗略查找的joint2高度貌似是0.077
            joint2_angle= math.atan2(z2, l)
            angle2.data = -joint2_angle + 1.57
            joint2pub.publish(angle2)
            

        except Exception as e:
            rospy.logerr("错误提示:%s",e)


        rate.sleep()
    # 6.spin    
    # rospy.spin()
