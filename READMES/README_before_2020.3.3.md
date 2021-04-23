Autoaim
=========
RM 2020 Autoaim Framework
----------------------------------
## Also this is for code backup.  
  
Recent change:  
> * 2020.2.23 Add ECO tracker (which is slow, 50fps)  
> * 2020.2.24 Add AimBase as the base class of AimVicinity and AimDistance, tested with simulator.   
>> * Some of the hpp files are moved to include/universal, e.g. NumRec.hpp Armordecision.hpp for  
>> universal purposes.  
>> * No virtual function added, considering efficiency.
> * 2020.2.25 -GQR update the knn and PCA data with some minor changes to ArmorPlate and GetPos.  
> * 2020.2.26 -GQR correct some fault in decision procedure -HQY: modifications, see the followings  
>> * Modification to ArmorPlate.hpp: use two dots to represent the light bar, and calculate the angle >> of the ligt bar. Using this angle(which has a meaningful +/- sign) to filter the tar_list.
>> * Change in: AimDeps: Add `float aver_ang `, change `cv::Point3f r_vec` into `cv::Mat r_vec`, for Armordecision.hpp of GQR, I have made some changes because of r_vec at `line 76`.  
>> * Also, the delay time of cv::waitKey is now changable, with `amp_debug`(specialized printf) being >> able to print colorful sentences to make debug reading easier.
> * 2020.2.27 & 2020.2.28 (morning)
>> * ### Quick note for Mr.GQR: Detective.hpp, AimVicinity.hpp, Armordecision.hpp have been changed!!
>>> * Detective.hpp attribute `_enemy_color` is made public `line  183`.
>>> * AimVicinity.hpp ```AimVicinity(const bool _blue = true);``` at `line 22` with mutiple changes in aimAuto.  
>>> * Armordeciison.hpp fixed problems at `line 74`, it was unable to make decision previously.
>> * Changes on AimBase  
>> * Add color preset and Aimdeps.hpp modification. Multiple params are moved to Aimdeps.hpp.
>> * Add ```init_color_blue = readParam<int>(nh, "/init_color_blue")>0 ? true : false;``` to get params from ros params.  
>>> * With roslaunch, params could be easily loaded with 
```
<rosparam command="load" file="$(find armor)/config/blue_color.yaml" /> /// in the blue.launch  
roslaunch armor blue.launch  
```
>>> * To run a single node e.g. Frame, use:
```
roscore && rosparam load <the path of blue_color.yaml in armor/config or red_color.yaml>
rosrun armor Frame
```
>>> * Start without loading .yaml file could trigger error.
> * 2020.2.29 (Morning 1:23)  
>> * Fixed Bugs from LightMatch, deleted lowExposure with the replacement from sampling by Dinnger.  
>> * Fixed problems of `GetPos.hpp` packUp function.  
>> * Add `LOG.hpp` into `ArmorPlates.hpp`, which can print colorful setences, making debug easier.  
```
///EXAMPLE FOR LOG.HPP rmlog::LOG::printc  
#define print rmlog::LOG::printc  
print(rmlog::F_RED, "This is the ", 20, "th time for test.");   
```
> * 2020.3.1 (Afternoon 15:10) -Made by GQR
>> * Correct the recognize module and it can function smoothly 
>> * Add all Sentry decision module @ `Sentrydecision/Sentrydecision.hpp`
>> * Add autoswitch module @ `AimBase.hpp` this module can automatically switch the current mode 
>>>* Tip:these two module base on #define SENTRYDECISION @`AimDeps.cc`.omit it if not need . 
>> * Add Type score in decision module 
>> * put all param i need in the Aimdeps.cc
> * 2020.3.1 -Made by Enigmatisms  
>> * Simulator.hpp and HPG(from grabing frames) update. Several functions for testing mode switching are added. Now Simulator can simulate the infomation from gimbals, for e.g. telling the miniPC to switch from distance Aim to vicinity Aim. Two video captures are used to display different videos. All tested.
>> * Found some bugs in AimBase.hpp `function: singleTracker`, and fixed it.
>> * Major changes in Frame.cc, modified the mode switching mechanism. Add reset functions to all three modes which are currently available.
>> * Modifications in `sampling.hpp`.  
> * 2020.3.2 Modifications
>> ### Re-organized `armor/include/universal`, `universal` is removed. Instead, several dirs are added, which is more specific.
>> ### GQR needs to merge almost all of the changes because of re-organization. `Armordecision.hpp`, `AimBase.hpp`, `GetPos.hpp` have actual logical changes.

# THIS FILE WILL NO LONGER BE DISPLAYED.
