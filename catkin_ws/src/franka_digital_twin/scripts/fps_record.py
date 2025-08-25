#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock

class FPSRecorder:
    def __init__(self):
        rospy.init_node('fps_recorder', anonymous=True)
        self.last_sim_time = None
        self.last_real_time = None
        self.frame_count = 0
        self.start_time = rospy.Time.now()
        rospy.Subscriber('/clock', Clock, self.clock_callback)

        
        self.log_file = open("gazebo_fps.log", "w")

    def clock_callback(self, data):
        
        sim_time = data.clock.to_sec()
        real_time = rospy.Time.now().to_sec()

        if self.last_sim_time is not None:
            # 计算 FPS
            sim_time_diff = sim_time - self.last_sim_time
            real_time_diff = real_time - self.last_real_time
            if real_time_diff > 0:
                fps = sim_time_diff / real_time_diff
                self.log_file.write(f"{real_time},{fps}\n")

        self.last_sim_time = sim_time
        self.last_real_time = real_time

    def run(self):
        rospy.spin()

    def __del__(self):
        self.log_file.close()

if __name__ == '__main__':
    try:
        recorder = FPSRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass
