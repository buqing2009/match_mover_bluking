# Match-Mover-Bluking
Alternative version with GRIC-PELC based keyframe selection  and tracking approach using Optical Flow between keyframes

`website:` [here](http://homepage.uibk.ac.at/~csak7995/matchmover/)

* Source file: [matchmover.tar.gz](http://homepage.uibk.ac.at/~csak7995/matchmover/matchmover.tar.gz)

* Camera calibration file: [calibration.xml](http://homepage.uibk.ac.at/~csak7995/matchmover/calibration.xml)
* Documentation with Doxygen also available as [.pdf](http://homepage.uibk.ac.at/~csak7995/matchmover/doxygen/refman.pdf)
* [Original video](http://www.kknd-xtreme.de/matchmover/teatime_deactivated_steady_shot_final.avi) (Uncompressed YUV 4:2:0 I420, 163.5 MB)
* [Final video](http://homepage.uibk.ac.at/~csak7995/matchmover/videos/teatime_deactivated_steady_shot_final_rendered.avi) with superimposed [Utah Teapot](http://en.wikipedia.org/wiki/Utah_teapot) (MPEG4, 16.4 MB)
* [Old video](http://homepage.uibk.ac.at/~csak7995/matchmover/videos/ski_vga_rendered.avi) with image sta bilisation shortened before critical frames (MPEG4, 4.9 MB)



## 注意事项
这个代码比较坑爹的地方有以下注意点：

* 编译没有`CMakeLists.txt`，按照教程只能添加eclipse等IDE中一个一个配置lib，include，需要自己写`CMakelists.txt`，**@bluking**已经针对ubuntu14.04下对opencv2.4.9版本写了一个`CMakelists.txt`，并且优化了其工程的结构
具体的依赖项如下：
	* GLUT/OPENGL: `sudo apt-get install freeglut3-dev`
	* OPENCV（2.4.9）: `源码安装，注意要安装nonfree模块！`
	* SDL（1.x）: `sudo apt-get install libsdl1.2-dev`
	* gtkmm-2.4: `sudo apt-get install libgtkmm-2.4-dev`

**P.S.** `SDL`和`gtkmm`版本可以安装**最新版本**，请自行修改`CMakeLists`和源码对应**头文件**

* 代码中有几处明显的错误(**本git上传的源码已经全部修正过，如有DIY者可以在**[here](http://homepage.uibk.ac.at/~csak7995/matchmover/matchmover.tar.gz)下载**原作者**源码进行修改！)：
	* 第一个只在`macosx`下才会出现因为`mac`下`opengl`的头文件和`linux`下不一样，如果在`linux`下没问题，如果是`mac`下自行修改头文件.(一般工程都会针对不同操作系统分开引头文件,作者太懒，考虑到连`CMakelists.txt`都懒得写就不多说了，不过源码注释是我看过的最人性化的，没有之一，那注释的**逼格**。。而且注释很丰富，基本每行都有，保证读者的理解)
	* 在`multiview.cpp`第455行开始，把所有的`for循环`中用到的迭代器变量`t`改为`i`，否则与后面的rt矩阵的`t`矩阵变量名重复，导致编译错误
	* 从官网上下载的`xml配置文件`，格式有错误需要在`文件头`上加xml格式标识`<?xml version="1.0"?>`，否则程序无法正确读取相机的标定参数矩阵
 
