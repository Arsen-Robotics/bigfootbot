import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
from pygame.locals import *

class KeyboardInputPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher_node')
        self.publisher_ = self.create_publisher(String, 'linear_act_command', 10)
        self.timer_ = self.create_timer(0.1, self.keyboard_callback)

        pygame.init()
        #self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption('Linear actuator control')
        pygame.mouse.set_visible(0)
     
    
    # Is responsible for handling keyboard events. 
    # When the 'e' (extend) or 'r' (retract) key is pressed, it creates a message with 
    # the corresponding key as data and publishes it to the 'linear_act_command' topic 
    # When any key is released, it creates a message with 's' (stop) as data and publishes it.
    def keyboard_callback(self):
        for event in pygame.event.get():
            if (event.type == pygame.KEYDOWN):
                if event.key == pygame.K_UP:
                    msg = String()
                    msg.data = 'r'
                    self.get_logger().info(f'Publishing: {msg.data}')
                    self.publisher_.publish(msg)
                elif event.key == pygame.K_DOWN:
                    msg = String()
                    msg.data = 'e'
                    self.get_logger().info(f'Publishing: {msg.data}')
                    self.publisher_.publish(msg)
                elif event.key == pygame.K_RIGHT:
                    msg = String()
                    msg.data = 'y'
                    self.get_logger().info(f'Publishing: {msg.data}')
                    self.publisher_.publish(msg)
                elif event.key == pygame.K_LEFT:
                    msg = String()
                    msg.data = 't'
                    self.get_logger().info(f'Publishing: {msg.data}')
                    self.publisher_.publish(msg)
            if (event.type == pygame.KEYUP):
                msg = String()
                msg.data = 's'
                self.get_logger().info(f'Publishing: {msg.data}')
                self.publisher_.publish(msg)
            if event.type == pygame.QUIT:
                pygame.quit()

def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher_node = KeyboardInputPublisher()

    try:
        rclpy.spin(keyboard_publisher_node)
    except KeyboardInterrupt:
        pass

    keyboard_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()