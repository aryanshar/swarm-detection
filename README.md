Code for the paper titled **Lightweight Multi-Drone Detection and 3D-Localization using YOLO**

---
# Drone Detection

## Setting up

1. Install the [Ubuntu](https://www.ubuntu.com/) Linux distribution.

2. Open terminal and enter the following lines to build [Darknet](https://pjreddie.com/darknet/):
```
git clone https://github.com/pjreddie/darknet.git
cd darknet
make
```

> **Note**: The folder data/dataset contains the image files and its corresponding annotation files in the same folder only. |

3. Move **weights**, **data**, **scripts**, **cfg** in the root directory of your cloned darknet.

4. Comment the corresponding cfg file you want to use, based on either test/train configuration as per mentioned in the comments of the cfg file.
5. Change lines 2 and 3 to your path in **data/obj.data**.
6. To do a train/test split, run the train_test_split.py in the scripts folder after changing the paths. The **train.txt** and **test.txt** files are generated inside the data folder. 

---

## Running

To run prediction on an image named input.png (sourced inside the darknet root folder), open terminal in the root directory of the **darknet** executable and enter:
```
./darknet detector test data/obj.data cfg/tiny-yolov4_drone.cfg weights/backup_yolov4-tiny/yolov4-tiny_drone_best.weights input.png

```
---
## Results








https://user-images.githubusercontent.com/28498326/136603974-a57d66f7-2b70-4975-b7c4-c6f10682add3.mov




---

# Depth Estimation
