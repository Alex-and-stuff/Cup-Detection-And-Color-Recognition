# Cup-Detection-And-Color-Recognition
playing around with openCV in C++ and trying to detect the color of cups placed on the field.

# Launch
type in 
```launch
$ roslaunch cupDetection cv_test.launch 
```
for changes, look into cv_test.c in the src folder and remember to catkin_make after changes are made

# Some notes 
still in progress, but its fun writing cv in C++. Basic contour detection and color masking is done, but i'm still figuring out how to filter out the outliers and detect the targeted 5 cups.
