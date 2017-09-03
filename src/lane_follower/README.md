# Car barrier detection


This node detects car barrier in 3 steps
 1. Filtering by color
 2. Filtering by shape
 3. Calculating angle of bar

#### subscribed topic
 * /image_calibrated_compressed(sensor_msgs/CompressedImage)
#### published topic
 * /car_barrier(std_msgs/String)


## Lane detection
The first thing to do is filtering original image using mask and this process will be done in HSV coordinate not in RGB one because HSV coordinate is more easy to select typical color range. Next, I conver filtered image to binary image to reduce processed data.


<div style="text-align:center"><img src ="http://skfk3416.dothome.co.kr/?module=file&act=procFileDownload&file_srl=405&sid=53d33d931af1237f043ea3d56f34c4bc&module_srl=193" /></div>

## PID control
I used 'blob' function to recognize red rectangular. There are many parameters in blob functino but I used these 4 parameter in my project.
 * Threshold
 * Area
 * Circularity
 * Convexity


<div style="text-align:center"><img src ="http://skfk3416.dothome.co.kr/?module=file&act=procFileDownload&file_srl=404&sid=ec0b7488689704caf03f9aa02aca5d85&module_srl=193" /></div>

## Exception
 1.  finding two outer points and drawing blue line
 2.  calculating angle of line using atan2 function
 3.  pobulish topics (if angle > 45: Stop, if angle < 45: Go)
<div style="text-align:center"><iframe width="560" height="315" src="https://www.youtube.com/embed/pVKA_5ddetc" frameborder="0" allowfullscreen></iframe>
