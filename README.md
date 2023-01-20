# RGBD-multiview_3D-registration-ICP-
This is the additional version for RGBD-multiview 3D-registration. (ICP algorithm)

- subject : RGBD-multiview 3D-registration
- equipment : MS Azure kinect
- library : pykinect, opencv, open3d, numpy
- source code\
  cal_RT.py : RGBD capturing, calculationg 3d transformation matrix (least squares method)\
  pc_combine.py : registration process of point-clouds using calculated transformation matrix\
  py_play.py : visualization of registrated point-clouds\
  remove_all.py : removing all of generated files from previous run\
  icp.py : using ICP(iterative closest point) method to improve matching accuracy
- files\
  pykinect_azure : pykinect library\
  color : captured color image\
  trns_color : color image mapped to depth map\
  pc : captured point-clouds (before registration)\
  rt : calculated transformation matrix\
  fin_pc : transformed point-clouds
 - how to test\
  add libraries\
  connect azure kinect\
  run cal_RT.py and pc_combine.py sequentially\
  if you need an increase in accuracy, run icp.py (it change RT matrix)\
  run pc_combine.py again
  
ICP\
  Applied icp algorithm using closest point, and used point of specific range to work properly.\
  (point of specific range : only use point that distance's in range 10<=d<20, it is more efficient than using all points)


- before/after ICP\
![1-1](https://user-images.githubusercontent.com/83062612/213607751-38c34c2d-752a-491c-87a6-faad0254d4c1.PNG)
![2-1](https://user-images.githubusercontent.com/83062612/213607773-1c42230a-afc2-40fc-bc86-a4b81658f7fb.PNG)
