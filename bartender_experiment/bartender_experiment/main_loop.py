from cognitive_processes.main_loop import MainLoop

from cognitive_processes_interfaces import msg
from std_msgs.msg import Bool




class BartenderExperimentMainLoop(MainLoop):
    """
    Main loop of the bartender experiment. It is a MainLoop whose trials end when the simulator
    publishes a finished signal, in addition to the end by number of trials of the base class.
    """

    def __init__(self, *args, **kwargs):
        """
        Constructor for the BartenderExperimentMainLoop class. The arguments are those of MainLoop.
        """
        super().__init__(*args, **kwargs)
        self.finished_signal = False

    def setup_control_channel(self):
        """
        Configures the control channel as in MainLoop and subscribes to the finished signal of the simulator
        (``/emdb/simulator/sensor/finished_signal``).
        """
        super().setup_control_channel()
        self.finish_signal_subscriber = self.create_subscription(
            Bool,
            '/emdb/simulator/sensor/finished_signal',
            self.finished_signal_callback,
            1,
            callback_group=self.cbgroup_perception
        )

    def finished_signal_callback(self, msg):
        """
        Stores the last finished signal received from the simulator.

        :param msg: Finished signal.
        :type msg: std_msgs.msg.Bool
        """
        if not self.finished_signal and msg.data:
            self.get_logger().info("DEBUG - Finished signal received.") 
        self.finished_signal = msg.data

    def world_finished(self):
        """
        Tells whether the world has finished, so that the main loop resets it.

        :return: The last finished signal received from the simulator.
        :rtype: bool
        """
        return self.finished_signal


