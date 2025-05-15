import cv2
import numpy as np
import math

from functools import lru_cache

@lru_cache
def create_text_image(*text_lines:list[str],
                        width:int=640, height:int=480, font=cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale:int=2, font_thickness:int=3, text_color=(255, 255, 255), 
                        background_color=(0, 0, 0), line_spacing:float=1.5, output_file=None):
    """
    Creates an image with a black background and centers multiple lines of text.

    Parameters:
    - text_lines (list of str): List of text lines to display.
    - width (int): Width of the image.
    - height (int): Height of the image.
    - font (OpenCV font): Font type for the text.
    - font_scale (float): Scale of the text.
    - font_thickness (int): Thickness of the text.
    - text_color (tuple): Color of the text in BGR (default: white).
    - background_color (tuple): Color of the background in BGR (default: black).
    - line_spacing (float): Multiplier for spacing between lines (default: 1.5).
    - output_file (str or None): If provided, saves the image to this file path.

    Returns:
    - numpy.ndarray: The generated image.
    """
    # Create a background image
    image = np.full((height, width, 3), background_color, dtype=np.uint8)
    
    # Calculate total height of all text lines including spacing
    line_heights = []
    for line in text_lines:
        (text_width, text_height), baseline = cv2.getTextSize(line, font, font_scale, font_thickness)
        line_heights.append(text_height + baseline)
    
    total_text_height = int(sum(line_heights) + (len(line_heights) - 1) * line_spacing * line_heights[0])
    
    # Start drawing from vertically centered position
    y = (height - total_text_height) // 2
    
    for i, line in enumerate(text_lines):
        (text_width, text_height), baseline = cv2.getTextSize(line, font, font_scale, font_thickness)
        x = (width - text_width) // 2  # Center the text horizontally
        cv2.putText(image, line, (x, y + text_height), font, font_scale, text_color, font_thickness)
        y += int((text_height + baseline) * line_spacing)  # Move to the next line
    
    # Save the image to a file if an output path is provided
    if output_file:
        cv2.imwrite(output_file, image)
    
    return image

def merge_frames(frames, output_size=(1920, 1080)):
    """
    Merge multiple video frames into one frame in a grid layout.

    Parameters:
        frames (list): List of frames (images in numpy array format) to merge.
        output_size (tuple): Size of the output frame (width, height).

    Returns:
        numpy.ndarray: Merged single frame.
    """
    if not frames:
        raise ValueError("No frames provided.")

    # Calculate grid layout if not provided
    n_frames = len(frames)
    grid_cols = math.ceil(math.sqrt(n_frames))
    grid_rows = math.ceil(n_frames / grid_cols)

    # Determine the size of each cell in the grid
    output_width, output_height = output_size
    cell_width = output_width // grid_cols
    cell_height = output_height // grid_rows

    # Resize frames to fit into grid cells
    resized_frames = []
    for frame in frames:
        resized_frame = cv2.resize(frame, (cell_width, cell_height))
        resized_frames.append(resized_frame)

    # Create a blank canvas for the output frame
    merged_frame = np.zeros((output_height, output_width, 3), dtype=np.uint8)

    # Place each resized frame into the canvas
    for idx, frame in enumerate(resized_frames):
        row = idx // grid_cols
        col = idx % grid_cols
        y_start = row * cell_height
        x_start = col * cell_width
        y_end = y_start + cell_height
        x_end = x_start + cell_width

        merged_frame[y_start:y_end, x_start:x_end] = frame

    return merged_frame
