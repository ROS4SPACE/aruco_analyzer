import rospy
import psutil
import os
from aruco_analyzer.msg import sys_resources

class CpuMemMonitor(object):
    def __init__(self):
        self.process = psutil.Process(os.getpid())
        self.cpu_percent = None
        self.memory_percent = None
        self.interval = 1
        self.cpu_count = psutil.cpu_count()
        self.pub = rospy.Publisher(
            '/aruco_analyzer/system_resources_used', sys_resources, queue_size=10)
        self.message = sys_resources()

    def run(self):
        while not rospy.is_shutdown():
            self.message.cpu = self.process.cpu_percent()/self.cpu_count
            self.message.mem = self.process.memory_percent()
            self.pub.publish(self.message)
            rospy.sleep(self.interval)


if __name__ == '__main__':
    rospy.init_node('test')
    cpumem = CpuMemMonitor()
    cpumem.run()
