#!/usr/bin/env python3
import base64
import numpy as np
from PIL import Image
import io

def occupancy_grid_to_base64(occupancy_grid, width=240, height=128):
    """
    Convert ROS2 OccupancyGrid to a base64 encoded string with 2 bits per pixel (4 colors)
    
    Args:
        occupancy_grid: nav_msgs/OccupancyGrid message
        width: Target width of the output image
        height: Target height of the output image
        
    Returns:
        base64 encoded string representing the image
    """
    # Extract map data from occupancy grid
    map_data = np.array(occupancy_grid.data).reshape(
        occupancy_grid.info.height, 
        occupancy_grid.info.width
    )
    
    # Convert occupancy values (usually -1, 0, 100) to 0-255 range
    # -1 (unknown) -> 192 (light gray)
    # 0 (free) -> 255 (white)
    # 100 (occupied) -> 0 (black)
    # Values in between will be interpolated
    img_data = np.zeros_like(map_data, dtype=np.uint8)
    img_data[map_data == -1] = 192  # Unknown -> light gray
    img_data[map_data == 0] = 255   # Free -> white
    img_data[map_data > 0] = 255 - (map_data[map_data > 0] * 255 // 100)  # Occupied -> dark
    
    # Create PIL Image and resize to target dimensions
    img = Image.fromarray(img_data).resize((width, height))
    
    # Quantize to 2 bits (0-3 values)
    img_np = np.array(img) // 64  # 0-255 → 0-3
    
    # Pack 4 pixels (2 bits each) into 1 byte
    flat = img_np.flatten()
    packed = bytearray()

    for i in range(0, len(flat), 4):
        chunk = flat[i:i+4]
        if len(chunk) < 4:
            chunk = np.pad(chunk, (0, 4 - len(chunk)))
        byte = (chunk[0] << 6) | (chunk[1] << 4) | (chunk[2] << 2) | chunk[3]
        packed.append(byte)
    
    # Encode to base64
    b64_encoded = base64.b64encode(packed).decode("ascii")
    
    return b64_encoded

def image_to_base64(image_path, width=240, height=128):
    """
    Convert an image file to base64 encoded string with 2 bits per pixel (4 colors)
    
    Args:
        image_path: Path to the image file
        width: Target width of the output image
        height: Target height of the output image
        
    Returns:
        base64 encoded string representing the image
    """
    # Load and resize image
    img = Image.open(image_path).convert("L").resize((width, height))
    
    # Quantize to 2 bits (0-3)
    img_np = np.array(img) // 64  # 0-255 → 0-3
    
    # Pack 4 pixels (2 bits each) into 1 byte
    flat = img_np.flatten()
    packed = bytearray()

    for i in range(0, len(flat), 4):
        chunk = flat[i:i+4]
        if len(chunk) < 4:
            chunk = np.pad(chunk, (0, 4 - len(chunk)))
        byte = (chunk[0] << 6) | (chunk[1] << 4) | (chunk[2] << 2) | chunk[3]
        packed.append(byte)
    
    # Encode to base64
    b64_encoded = base64.b64encode(packed).decode("ascii")
    
    return b64_encoded
