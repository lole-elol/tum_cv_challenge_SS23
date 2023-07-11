# TUM Computer Vision Challenge SS2023

## Usage

The GUI can be started by running `main.m`. This will open a window where you can choose images of a room. The program will then reconstruct a 3D Model of this room. See below for a detailed description of the workflow.

### Loading Data

In order to create a 3D model of a room you need to upload images of this room in JPG format. You can do this by pressing the **Import** button.

The file explorer will open and you can choose a folder containing a subfolder `images` with all the images of your room. The folder should also contain a subfolder `parameters` with camera parameters (`cameras.txt`)

### 3D Reconstruction

When all images are imported you can start reconstruction by pressing **Reconstruct Room**. This may take some time.

### Interacting with the model

The 3D model of the room will be displayed as soon as the reconstruction finished. It wil show the floor and ceiling as well as objects in the room as cuboids. You can explore the model from different direction by panning and look at details by zooming with your mouse wheel.

If you want to take a look at textures or get a more detailed view of the room you can press **Show Pointcloud**. This will display the 3D pointcloud which was generated from the images.

### Measuring

If you want do measure distances in the finished model, we need some information from you in order to scale the 3D model correctly. Please enter the height of the room into the provided textfield before pressing **Reconstruct Room**.

When the 3D reconstruction is finished, distances in the model can be measured. After activating the *measure* mode you can just click onto points in the model and the distance will be displayed.

## Toolboxes:
(command: `ver`)
MATLAB                                                Version 9.14        (R2023a)
Computer Vision Toolbox                               Version 10.4        (R2023a)
Image Processing Toolbox                              Version 11.7        (R2023a)
Lidar Toolbox                                         Version 2.3         (R2023a)
Medical Imaging Toolbox                               Version 1.1         (R2023a)
Signal Processing Toolbox                             Version 9.2         (R2023a)
Statistics and Machine Learning Toolbox               Version 12.5        (R2023a)