# Basic Line Detector
Python OpenCV, a basic script that detects lines with low curvature then calculates angle and error relative to line.

<b>Step 1</b>
Convert 3-channel image to binary image by thresholding.

<b>Step 2</b>
Denoise image with OpenCV's morphology operators.

<b>Step 3</b>
Find all contours as a list.

<b>Step 4</b>
Find the contour with maximum ratio. This logic is based on lines have high ratios.

<b>Step 5</b>
To calculate average angle of line, OpenCV's minAreaRect function used. It contains all necessary data.

<b>Step 6</b>
Draw output data on screen.
