# Depth Calibration
 
## Run the program
- Navigate to the build directory and launch the executable
- Or open a terminal in the build directory and run the sample :

      ./cameraCalibration  svo_file.svo

### Features
 - Press 'f' to jump forward in the video (optional)
	- Press 'b' to jump backward in the video (optional)
	- Press 'w' to wash everything
	- Press 's' to save the video as an image list in all different displays
	- Press 'c' to save the images from the ChArUco detection (optional but always after s)
	- Press 'p' to launch the SFM pipeline
	- Press 'd' to compare the depth given by the ZED camera software and the depth calculation using all the previous data
	- Press 'q' or 'esc' to exit...
	- Note that the order should be : w -> s -> p -> d , the rest are optional
  
