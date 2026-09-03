from cognitive_processes.main_loop import MainLoop

from cognitive_processes_interfaces import msg
from std_msgs.msg import Bool




class BartenderExperimentMainLoop(MainLoop):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.finished_signal = False

    def setup_control_channel(self):
        super().setup_control_channel()
        self.finish_signal_subscriber = self.create_subscription(
            Bool,
            '/emdb/simulator/sensor/finished_signal',
            self.finished_signal_callback,
            1,
            callback_group=self.cbgroup_perception
        )

    def finished_signal_callback(self, msg):
        if not self.finished_signal and msg.data:
            self.get_logger().info("DEBUG - Finished signal received.") 
        self.finished_signal = msg.data

    def world_finished(self):
        return self.finished_signal


