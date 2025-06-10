#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pzb_msgs.msg import VoiceCommand
import json
import os

class VoiceCommandClass(Node):
    def __init__(self):
        super().__init__('pzb_voice_node')
        
        # Create publisher
        self.publisher = self.create_publisher(VoiceCommand, 'voice_commands', 10)
        
        # Initialize current command state to UNKNOWN 
        # self.current_command = VoiceCommand.UNKNOWN
        self.current_command = VoiceCommand.UNKNOWN

        
        # JSON file path (relative to script location)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.json_file_path = os.path.join(os.path.dirname(__file__), 'VoiceLibs', 'command.json')
        
        # Create timer for 1Hz publishing
        self.timer = self.create_timer(1.0, self.publish_command)
        
        self.get_logger().info('PZB Voice Node has started - initial state: STOP')

    def publish_command(self):
        # Check if JSON file exists and read it
        if os.path.exists(self.json_file_path):
            self.get_logger().info(f"File exists")
            try:
                with open(self.json_file_path, 'r') as file:
                    json_data = json.load(file)
                    
                # Log the JSON content
                self.get_logger().info(f'Voice recognition result: {json_data}')
                
                # Extract the command from JSON and update current state
                if json_data:
                    command_word = list(json_data.keys())[0]
                    
                    # Map word to VoiceCommand constant
                    command_mapping = {
                        'backward': VoiceCommand.BACKWARD,
                        'begin': VoiceCommand.BEGIN,
                        'down': VoiceCommand.DOWN,
                        'finish': VoiceCommand.FINISH,
                        'forward': VoiceCommand.FORWARD,
                        'left': VoiceCommand.LEFT,
                        'right': VoiceCommand.RIGHT,
                        'rotate': VoiceCommand.ROTATE,
                        'stop': VoiceCommand.STOP,
                        'up': VoiceCommand.UP,
                        'unknown': VoiceCommand.UNKNOWN# Default fallback
                    }
                    
                    if command_word in command_mapping:
                        # Update current command state
                        self.current_command = command_mapping[command_word]
                        self.get_logger().info(f'Updated voice command to: {command_word.upper()}')
                        
                        # Remove the file after successful processing
                        os.remove(self.json_file_path)
                    else:
                        self.get_logger().warn(f'Unknown command received: {command_word}')
                        
            except json.JSONDecodeError:
                self.get_logger().error(f'Failed to parse JSON file: {self.json_file_path}')
            except Exception as e:
                self.get_logger().error(f'Error reading file {self.json_file_path}: {str(e)}')

        # else:
        #     self.get_logger().info(f"File does not exists {self.json_file_path}")

        
        # Always publish current command state (whether updated or previous)
        msg = VoiceCommand()
        msg.command = self.current_command
        self.publisher.publish(msg)
        
        # Convert command number back to name for logging
        command_names = {
            VoiceCommand.BACKWARD: 'BACKWARD',
            VoiceCommand.BEGIN: 'BEGIN',
            VoiceCommand.DOWN: 'DOWN',
            VoiceCommand.FINISH: 'FINISH',
            VoiceCommand.FORWARD: 'FORWARD',
            VoiceCommand.LEFT: 'LEFT',
            VoiceCommand.RIGHT: 'RIGHT',
            VoiceCommand.ROTATE: 'ROTATE',
            VoiceCommand.STOP: 'STOP',
            VoiceCommand.UP: 'UP',
            VoiceCommand.UNKNOWN: 'UNKNOWN'
        }
        
        command_name = command_names.get(self.current_command, 'UNKNOWN')
        self.get_logger().info(f'Published: {command_name} ({self.current_command})')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandClass()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
