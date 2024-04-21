# std lib
from typing import List

# rclpy lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class FloatingPlatformGPIO:
    def __init__(self, device:str="jetson", gpio_mode:str="BOARD", gpio_ids: List[int]=None)->None:
        """
        device (str): Device type (jetson or rpi).
        gpio_mode (str): GPIO mode (BCM or BOARD).
        gpio_ids (List[int]): List of GPIO pin numbers.
        """
        self.gpio = None
        if device == "jetson":
            import Jetson.GPIO
            self.gpio = Jetson.GPIO
        elif device == "rpi":
            import RPi.GPIO
            self.gpio = RPi.GPIO
        else:
            raise ValueError("Invalid device. Choose either 'jetson' or 'rpi'.")
        
        if gpio_mode == "BCM":
            self.gpio_mode = self.gpio.BCM
        elif gpio_mode == "BOARD":
            self.gpio_mode = self.gpio.BOARD
        else:
            raise ValueError("Invalid gpio mode. Choose either 'BCM' or 'BOARD'.")
        
        self.gpio_setup(gpio_ids)

    def gpio_setup(self, gpio_ids: List[int])->None:
        """
        gpio_ids (List[int]): List of GPIO pin numbers.
        """
        self.gpio.setwarnings(False)
        self.gpio.setmode(self.gpio_mode)
        
        self.gpio_ids = gpio_ids

        for gpio_id in self.gpio_ids:
            self.gpio.setup(gpio_id, self.gpio.OUT)

    def cleanup(self):
        self.gpio.cleanup()

    def valves(self, valve_ids, states):
        """
        valve_ids (List[int]): List of GPIO pin numbers.
        states (List[int]): List of states (0 or 1).
        """
        print(states)
        self.gpio.output(valve_ids, [1 - i for i in states])

class FloatingPlatformDirectValveControl(Node):
    def __init__(self):
        """
        GPIO pins (List[int]): (bearing, valve1, valve2, valve3, valve4, valve5, valve6, valve7, valve8)
        """
        super().__init__('fp_valve_control_node')
        # Register parameter
        self.register_param()
        # Initialize GPIO and valve
        self.gpio = FloatingPlatformGPIO(self.get_param("device"), self.get_param("gpio_mode"), self.get_param("gpio_ids"))
        self.gpio.valves(self.gpio.gpio_ids, [0,0,0,0,0,0,0,0,0])
        # Register subscriber
        self.subscriber = self.create_subscription(Int16MultiArray, self.get_param("topic_name"), self.valve_callback, 10)
    
    def register_param(self):
        """
        Register rosparams.
        """
        self.declare_parameter('topic_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('device', rclpy.Parameter.Type.STRING)
        self.declare_parameter('gpio_ids', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('gpio_mode', rclpy.Parameter.Type.STRING)
    
    def get_param(self, name):
        """
        Retrieve parameter value given its name.
        Args:
            name (str): name of parameter.
        """
        return self.get_parameter(name).value
        
    def valve_callback(self, msg):
        """
        Subscriber callback function.
        Args:
            msg (std_msgs/ByteMultiArray): ros2 message.
        """
        self.gpio.valves(self.gpio.gpio_ids, list(msg.data))
    
    def on_shutdown(self):
        """
        Custom shutdown hook.
        It turns off air-bearing and valves.
        """
        self.gpio.valves(self.gpio.gpio_ids, [0,0,0,0,0,0,0,0,0])

def main(args=None):
    rclpy.init(args=args)
    valve_control_node = FloatingPlatformDirectValveControl()
    
    try:
        rclpy.spin(valve_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        valve_control_node.on_shutdown()
        valve_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()