# RoboCup Junior Rescue Maze
Maze-solving robot that can detect heat, text, and color victim as part of the RoboCup Junior Rescue Maze robotics competition
## How it works
### Maze solving
We use the DFS algorithm to explore the maze.
### Detection
* Heat victims: through using the MLX contactless temprature sensor
* Color victims: through computer vision color segmentation algorithm to the incoming frames of the raspberry pi camera stream.
* Text victims: through performing OCR using Tesseract python wrapper, after manipulating the image using OpenCV
### Locomotion
We use 4 wheel differential drive by controlling the motors via L298N motor drivers. To facilitate autonomous navigation, we use optical encoders to detect the number of ticks on each wheel. To ensure accurate detection of the wheel ticks, we trigger an arduino interrupt for each change the encoder detects. 
