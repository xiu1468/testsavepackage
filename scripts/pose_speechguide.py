#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler

class SquarePathNode:
    def __init__(self):
        self.current_pose = None
        self.subscriber = rospy.Subscriber("/amcl_pose", PoseStamped, self.pose_callback)

        # 用戶定義的正方形端點坐標
        self.endpoints = [Point(0, 0, 0), Point(0, 0, 0), Point(0, 0, 0), Point(0, 0, 0)]
        self.current_endpoint = 0

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if self.current_pose is not None:
                current_point = self.current_pose.position

                # 如果當前姿勢已經到達端點，更新當前端點的索引
                if self.current_endpoint < 4 and self.is_close_to(current_point, self.endpoints[self.current_endpoint], 0.1):
                    self.current_endpoint += 1

                # 如果當前姿勢已經回到起點，提示完成
                if self.current_endpoint == 4 and self.is_close_to(current_point, self.endpoints[0], 0.1):
                    rospy.loginfo("恭喜完成正方形路徑!")
                    break

                # 提示下一步的行動
                rospy.loginfo("請執行以下步驟:")
                if self.current_endpoint == 0:
                    rospy.loginfo(" 1. 向前走1米")
                    rospy.loginfo(" 2. 向右轉90度")
                elif self.current_endpoint == 1:
                    rospy.loginfo(" 3. 向前走1米")
                    rospy.loginfo(" 4. 向右轉90度")
                elif self.current_endpoint == 2:
                    rospy.loginfo(" 5. 向前走1米")
                    rospy.loginfo(" 6. 向右轉90度")
                elif self.current_endpoint == 3:
                    rospy.loginfo(" 7. 向前走1米")
                    rospy.loginfo(" 8. 向右轉90度")

            rate.sleep()

    def is_close_to(self, point1, point2, distance_tolerance):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance < distance_tolerance

if __name__ == '__main__':
    rospy.init_node('square_path_node')
    node = SquarePathNode()

    # 讀取用戶定義的端點坐標
    for i in range(4):
        x = rospy.get_param("~endpoint_{}_x".format(i+1), 0)
        y = rospy.get_param("~endpoint_{}_y".format(i+1), 0)
        yaw = rospy.get_param("~endpoint_{}_yaw".format(i+1), 0)
        self.endpoints[i] = Point(x, y, 0)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.endpoints[i].orientation = Quaternion(*quaternion)

    node.run()

