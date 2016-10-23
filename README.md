# NEEDLE INSERTION VREP SIMULATOR - README
This is a shared project between Robotics 2 and Medical Robotics. It consists in a VREP simulation of a needle penetration performed by a KUKA LWR 4+ teleoperated by a Geomagic Touch.

### Requirements
- OpenHaptics and **Geomagic Suite** for Setup the connection PC -> Haptic Device
- **VREP** (only on Windows and Linux), link [here](http://www.coppeliarobotics.com/downloads.html)
- **CHAI3D** library, downloadable from [here](http://www.chai3d.org/).
- Visual Studio **2015** (tested on Express, but should work also with the Community version)

### Build
In order to build the solution, you have to go through the following steps:

1. Unzip CHAI3D
2. Follow its guide to build the library
3. Clone the repository into `<path_to_CHAI3D>\CHAI3D\modules\` folder
4. Open the file `CHAI3D-V-REP-VS2015.vcxproj` with VS2015
5. Change the output directory of the project (`ALT + F7` and then *Properties*) with VREP root folder in which all the external plugin are located (something like `C:\Program Files (x86)\V-REP3\V-REP_PRO_EDU`)
7. Build the solution using `CTRL + SHIFT + B` in **RELEASE** and **x86** mode. Obviously VREP must not be open, otherwise VS will fail the build.

### Usage
Once you have successfully built the solution, you can open the scene `robot_scene.ttt` with VREP.
There are two UIs:

- The first one allows you to enter the properties of the **tissue**, like *K, B, thickness* and the *depth* of each layer after which you will perforate it (expressed in percentace - so a number in [0.0f, 1.0f] )
- The remaining one allows you to set up the **teleoperation** scheme from the 3 available.

Notice that firstly you have to setup the layer parameter, then you setup the teleoperation and in order **to start the simulation, you must press PLAY from the second UI**, otherwise the scene won't work properly.
There is a VREP bug that did not allow to restart the simulation after you stopped it, so if you want to re-run the simulation you have to close VREP, re-open the scene and start again the simulation (it's not our fault :) ).

If during the simulation you want to print to console something, it will be shown in the VREP console.

Remember that the Geomagic Touch is able to provide 3.3 N of maximum force, so **the forces you send to the device should never exceed 3.0 N**, otherwise you can broke the haptic device (CHAI3D should automatically saturate the forces, but we live in a probabilistic universe, so everything can happen :/)
