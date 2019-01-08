## Project: Search and Sample Return

---

**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/warped.png
[image2]: ./misc/masked.png
[image3]: ./misc/gold.png 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Perspective Transform.
As discussed on the provided video I added the mask to identify the obstacles on the warped image.

The same function is reflected in the `perception.py` file and used by the `perception_step(Rover)` function in order to populate the obstacle map like `obs_map = np.absolute(np.float32(threshed) -1) * mask`

![alt text][image1]

#### 2. Color Thresholding

Using the mask returned by the previous function I can separate 2 different maps, one with the navigatable area (left) and another one with the obstacles (right)

In the `perception.py` file I use this 2 maps to populate the `Rover.vision_image`, I use the blue channel for the empty space `Rover.vision_image[:,:,2]=threshed * 255` and the Red channel for the obstacles `Rover.vision_image[:,:,0] = obs_map * 255`

![alt text][image2]

#### 3. Impose range
As per the discussion of the [live feed](https://www.youtube.com/watch?v=-L0Fz4UnlC8) and confirmed during my tests is better to imposse a range on the perception step in order to increase the map accuracy by eliminating the pixels further from the rover as the transformation tends to get distorted.

In the `perception.py` file I use the range impose function while after converting the image pixel values to rover-centric coords for both the empty space map and the obstacles map.

```
clear_xpix, clear_ypix = rover_coords(threshed)
clear_xpix, clear_ypix = impose_range(clear_xpix, clear_ypix)
obs_x_pix, obs_y_pix = rover_coords(obs_map)
obs_x_pix, obs_y_pix = impose_range(obs_x_pix, obs_y_pix)
```
#### 4. Find the golden ~~tickets~~ samples
The function to find gold is based on the same principle of the `color_thresh` function but with the levels adjusted for the yellow of the samples.

![alt text][image3]

#### 5. Distance to
This function provides the estimated distance between 2 coordinates in a plane.

I use this function on the `decision.py` file in several places on the `decision_step` function for comparing distances and taking decision like when the rover leaves the starting point in order to activate the offset to stick to the left wall, or after picking up a sample in  order to go back to the place it was before the attempted picking so the map can resume from the same place, etc.

#### 6. Process stored images. 
On this step I stitch all the functions together in order to do a rough prototype to be used on the `perception.py` file on the `perception_step` function later on.

Based on the previous lesson I defined the destination size as 5 and the bottom offset as 5px. Following on the same samples as the lessons the perspective transform is the same as explained before.

The perspective transoform is a little bit different to reflect the obstacle mask, following this I applied the color thresh to the navigatable terrain as in the lesson and then applied the mask to it in order to get the obstacle thresh.

Once both of the images are created we can then applie the clear pixels to the blue channel and the obstacles to the red channel.

Again the convertion of the pixels to rover-centric coordinates is the same as in the lessons with the small difference of the imposed range and the same techinique was applied to the obstacle pixels.

Converting the rover-centric values to world coordinates was the same like in the lesson and applied the same concept for the obstacles.

Following the conclussions on the live feed I updated the map only when the rover's pitch and roll was close to 0 to increase the fidelity of the map.

I then check for the gold samples on the images and if present applied the same functions as before and marked the pieces on the green channel.

Finally just built the image to display on the video presenting the threshed image as the rover navigates.

### Autonomous Navigation and Mapping

The Rover simulation is launched using  a resolution of '1024 x 768' and Graphics quality of 'good'.

#### 1. Perception, drive_rover, decision

I use the same change on the `perspect_transform` function in order to return the mask together with the warped image for using on issolating the obstacles later on.

I also added the new functions of `impose_range` and `find_gold`,  then filled up the `perception_step` function in a similar way as in the notebook with the difference of saving the naigation angles and distances for the navigation area, the obstacles and the gold samples on the Rover object for use on the decision step later on.

---
On the `RoverState` class inside the `drive_rover.py` file I added some support variables to keep track of things like the time the rover has been on each state (gold, stuck, spinning, unstack, etc), I also keep track of the unstuck method I am trying, the starting position, the position the rover was before attempting to collect the samples, etc.

---
On the `decision_step` inside the `decision.py` I added the new function `distance_to` as explained on the notebook for messuring the distance between 2 points on the map.

As discussed on the feed the easiest way to map the simulated environment is to "hug" on of the walls and following the concensus I also opted for hugging the left wall. 

In order to achieve this I added a possitive offset to the mean angle and following some advices applied it to the standard deviation to achieve a constant distance from the wall rather than just ossilating left and right on big angles.

---
**Improvement point**

This approach only works on this particular environment because the rover is in a contained space where it can't go out of and it will eventually make a loop going in circles, however in a real situation this would be a very bad idea, specifically without knowing beforehand the layout where the rover currently is.
A better aproach given the time would be to implement a path finder algorithm (eg: A*) and divide the area into grids to provide a restriction to the area that can potentially be mapped, the rover can then start to explore the grids one by one chasing after the unknown cells on each grid using the pathfinder function. This will also be applied to the samples once recogniced making sure that the approach path is as straight as possible to avoid getting stuck close to walls or obstacles.

---

I first verify if the rover collected the 6 samples and if it is close from the starting point, if both are true then it will flag the rover as returning home `Rover.going_home = True` to turn off the wall hugging function and be able to get to the middle of the map.

##### forward mode

The next check I perfom is to monitor if the rover is in 'forward' mode but the but the ground speed is close to 0.0, if this is true it means that the rover is most likely stuck and it will enter into the 'stuck' mode, this check is performed every 15 seconds.

In order to avoid spinning in circles on open spaces because of the offset I added a monitoring function and will turn off the `hug`option if it has been spinning for more than 800 seconds, it will then turn it on again after 200 seconds so it can continue mapping with the wall to it's left.

After confirming that the rover is not hard spinning I will reset the spin check timer to perform the chek later on.

**sample collection**

The next is to verify if the rover is near a sample and change the state to 'gold' mode in order to peform the picking actions.
If the rover has `gold_angles` and is closer than 50 mts from the sample it will start the preparations to aproach the samples and once we are closer than 40 mts it will change the mode to 'gold' to try the picking process.

**navigtion**

If the rover is not 'stuck' or spinning or ready to collect sampless then it can just keep on navigating, for that the default options are more less the same with the small difference that I will check for the `going_home` flag in order to use or ignore the offset to add for the angles.

```
Note: I tried removing the offset for sharp right turns in the hope of improving the obstacle avoidance part but more testing is needed and a better obstacle marking is prefered, perhaps marking the cells with cost of movement on the obstacles and decreasing it as it goes further away but that would also require a more complex route planner algorithm.
```

The last part added by me on the forward section was to verify if the rover is going home and if it at least 5 mts away from the starting position in order to chahe the rover's mode to 'home' wehre it will just stop and wait.

```
Personal note: perhaps some fireworks are needed for this mode.
```

##### stuck mode

If the rover enters this mode it will try a series of methods to try and get itself unstuck, the methods will go in order of severity and it will reset at the end if nothing worked in thehope that further attemps will be more successful. I found that due to some glitches in the simulator environment it tends to get stuck when is veraly close to some of the rocks (perhaps is quick sand rather than a glitch) so one of the un-stuck methods will surelly work, so far I have not encountered any situation (finger crossed) that will keep the rover stuck forever.
Bellow is the explanation for each hof the methods in the same order tried by the rover.

**0: reverse**

The first method will have the rover simply reverse on a straight line, this method is specially usefull when it hit some small rocks that the camera can't see but is not really stuck too much.
the rover will reverse for a total of 150 seconds to try and get enough clearance to continue.

---
**Improvement point**

This method could be more robust if it actually monitors if the rover is reversing and if the obstacle is ahead meaning that it can stop rather than just blindly keep on reversing for the given ammount of time.

---

**1: small steering**

The rover will try to spin in place for a small amount of time, if it can see empty space to either left or right it will spin in that direction, if not it will default to spinning to the right. This spin will continue untill the rover can see empty terrain or 100 seconds passed trying.

**2: right spinning**

The rover will spin to the right for 20 seconds.

**3: full speed backwards**

The rover will attempt to just go full throttle bacwards for 80 seconds.

---
**Improvement point**

This method could also  use the same inmprovement as the reverse method to just move untill clear terrain is on sight.

---

**4: full speed ahead**

Same as before but forward in cases where some strange angled stone got one of the back wheels of the rover.

**reset**

If everything failed it will just reset the state of the stuck function and put the rover into 'stop' mode in the hopes that another go will do the trick.

##### gold mode

The rover will monitor the time spend on thhis state to avoid staying trying to pick gold forever, if it has been in this state for over 80 seconds it will flag itself as stuck and hopefully it will be able to pick the sample next try.

---
**Improvement point**

As mentioned before it would be better to implement a pathfinder algorithm and approach the sampleas in a straight line by positioning the rover neext to the sample and spinning towards it instead of the current method that just adjuste the steering angle to get to it.

---

First, if the rover is near the sample it will step on the brakes and pick up the sample.
If the rover has the gold sample on sight it will adjust the navigation angle pointing to the rock and approach it.

After picking up the sample the rover will try to reverse towards the point where it was when starting the pick attempt in order to continue the mapping process. This step could also benefit with the pathfinder algorithm as it can return to the exact spot and yaw before resuming the mapping.

After finishing theprocess the rover will verify if indeed it picked up the sample by comparing the pre pick up count with the current one, if the picking was succesfull it will continue mapping, however if is not it will reverse in hopes of re targetting the sample again for another try.

##### home mode

Once the rover is home it will just stop and await for further instructions.

#### 2. Launching in autonomous mode.

The rover can map over 90% of the area with a fidelity of over 70%, most of the times is succesful in picking up all the 6 sample rocks, a full run video can be seen [here](https://www.youtube.com/watch?v=nMEA3PFA9a4)

While the mapping part is pretty robust, there are some cases where the rover will fail at finding the 6 sample rocks so it can only find and collect 5 of them,   I have had a hard time myself manually navigating and finding the last sample with many instances where I just have to give up because it does not seem to appear anywhere.

Another problem is that sometimes the rocks will be hidden just after a big rock on the right side and the rover will mark it on the map but fail to collect it due to the sharp turn that it will have to do, for this cases I can either use a more robust approach at picking up the samples where the rover can spin in place to face the sample and collect it or swap the offset at the end if it does not have all the rocks in order to hug the right wall instead and go against the previous path.

