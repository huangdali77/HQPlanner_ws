#!/usr/bin/env python3

import rospy
from hqplanner.msg import ref_line_test
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point

def reflineCallback(msg):
    x =[]
    y = []
    for point in msg.path:
        x.append(point.x)
        y.append(point.y)
    return x,y

# def ref_line_sub():
#     x,y=rospy.Subscriber("/ref_line_points", ref_line_test, reflineCallback)
#     plt.plot(x, y)
#     plt.show()TypeError: cannot unpack non-iterable Subscriber object

if __name__ == '__main__':
    rospy.init_node('ref_line_sub', anonymous=True)
    rospy.Subscriber("/ref_line_points", ref_line_test,
                            reflineCallback, queue_size=1)

    # plt.plot(x,y)
    # plt.axis('equal')
    # plt.show()
    rospy.spin()





# class ref_line_test_sub_and_plot():
#     def __init__(self):
#
#         self.path_subscriber = rospy.Subscriber("/ref_line_points", ref_line_test,
#                                                 self.ref_line_test_callback, queue_size=1)
#
#     def ref_line_test_callback(self, ref_line):
#         assert isinstance(ref_line, ref_line_test)
#         xs = []
#         ys = []
#         for point in ref_line.path:
#             xs.append(point.x)
#             ys.append(point.y)
#         return xs, ys
#
#     # def plot(self):
#     #     plt.plot(self.xs[0:100],self.ys[0:100])
#     #     plt.axis('equal')
#     #     plt.show()
#
#         # print(self.xs,self.ys)
#
#
#
# # def main():
# #     rospy.init_node('ref_line_sub', anonymous=True)
# #     ref_line_test_sub_and_plot()
# #     rospy.spin()
#
#
#
#
# if __name__ == '__main__':
#     rospy.init_node('ref_line_sub', anonymous=True)
#     ref_line=ref_line_test_sub_and_plot()
#     ref_line
#     rospy.spin()
#
