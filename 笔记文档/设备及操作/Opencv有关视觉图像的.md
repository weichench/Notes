

## 有关图像的知识点积累

+ 单通道图像指的是，单个灰度值，，，，彩色图为三通道

+ ```
  数据类型
  Vec3f v3f = mat.at<Vec3f>(y, x);//read color values for pixel (y,x)     访问指定像素的色彩值
  Vec2f v2f(x0,x1); 
  Vec6d v6d(x0,x1,x2,x3,x4,x5); 向量的构造
  
  点的定义
  CV::Point pt1, pt2;  pt1.x, pt1.y
  
  由收尾端点画直线  cv::line(outputImg, pt1, pt2, CV_RGB(0, 0, 0), 1, 8)
  
  
  ```

  ### imshow()函数
  
  到名称为name的窗口体，找不到就自己创建一个窗口 --> 会有一个判断（如果window==NULL 或者  arr==NULL就exit(0)） --> 设置图片与窗口的属性，将二者属性设为一致  -->还有一个判断（如果窗口体大小发生变化，就更新窗体）
  
  **该函数需要消耗大约6ms的时间**
  
  ## waitkey()函数
  
  waitkey(0);  等待按键按下，程序一直在此无限延迟
  
  **imshow()**   ，在窗口显示图形，先查找是否有同名的窗口，单纯的显示，相当于指令，没有再次在该窗口显示图像的指令时，图像会一直显示着，，waitkey（）的存在只是让imshow()的执行有一定缓冲，与显示无关



### 有关keylin

| Public Member Functions                                      |                                                              |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
|                                                              | [KeyLine](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#aba88c983c0480a8e96357c12b427aabd) () |
|                                                              |                                                              |
| [Point2f](https://docs.opencv.org/trunk/dc/d84/group__core__basic.html#ga7d080aa40de011e4410bca63385ffe2a) | [getEndPoint](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a88b18e563b54841e2642566c0851d520) () const |
|                                                              |                                                              |
| [Point2f](https://docs.opencv.org/trunk/dc/d84/group__core__basic.html#ga7d080aa40de011e4410bca63385ffe2a) | [getEndPointInOctave](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a30b2827d83df0e723a23946a2d3c37c1) () const |
|                                                              |                                                              |
| [Point2f](https://docs.opencv.org/trunk/dc/d84/group__core__basic.html#ga7d080aa40de011e4410bca63385ffe2a) | [getStartPoint](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a5a08a6f3f00e2997150988203426273b) () const |
|                                                              |                                                              |
| [Point2f](https://docs.opencv.org/trunk/dc/d84/group__core__basic.html#ga7d080aa40de011e4410bca63385ffe2a) | [getStartPointInOctave](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#ab2206aae58c7d9728b6a993be0a608ca) () const |
|                                                              |                                                              |

| Public Attributes                                            |                                                              |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| float                                                        | [angle](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a041a6bbf2fc22e3ab9f5a80ef5a5b922) |
|                                                              |                                                              |
| int                                                          | [class_id](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a4a82cb1083a3e18d54b055a692b60eee) |
|                                                              |                                                              |
| float                                                        | [endPointX](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a43b634f8f6b5953f2491b00bf6c231f6) |
|                                                              |                                                              |
| float                                                        | [endPointY](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a1b528bf3d1f3ab902e279ba1b5a593f1) |
|                                                              |                                                              |
| float                                                        | [ePointInOctaveX](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a260a577f6dc5778ef2fc668c5df32eb7) |
|                                                              |                                                              |
| float                                                        | [ePointInOctaveY](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a1f16ffeca01cfc6bfd9cec4c925c4248) |
|                                                              |                                                              |
| float                                                        | [lineLength](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a7a1b7b2712a9c77d64d749729bdb79c2) |
|                                                              |                                                              |
| int                                                          | [numOfPixels](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a5ef7fe33dd5d4096e679148c50bdb64e) |
|                                                              |                                                              |
| int                                                          | [octave](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#acd62d95914b8005ee4f425c171c2ed1f) |
|                                                              |                                                              |
| [Point2f](https://docs.opencv.org/trunk/dc/d84/group__core__basic.html#ga7d080aa40de011e4410bca63385ffe2a) | [pt](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a9502f2f63fc1d4d15d10730359e750f5) |
|                                                              |                                                              |
| float                                                        | [response](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a6cd22aa3cdcf0613f457d4881b51f7ad) |
|                                                              |                                                              |
| float                                                        | [size](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a3d23124ae5d210ba5ba5faebf5e89f3c) |
|                                                              |                                                              |
| float                                                        | [sPointInOctaveX](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#aadf42087a0908a900d34d9396037dc28) |
|                                                              |                                                              |
| float                                                        | [sPointInOctaveY](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a4d7fb8e09d606597d0af5a0df7e72087) |
|                                                              |                                                              |
| float                                                        | [startPointX](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#a300d7487fced03e0f781afbad5f6e67d) |
|                                                              |                                                              |
| float                                                        | [startPointY](https://docs.opencv.org/trunk/d1/dd7/structcv_1_1line__descriptor_1_1KeyLine.html#ac36c0fe32df87388d9fa6a8def75f7e4) |







/home/chenchen/Dataset/V1_02_medium/mav0/cam0/data/1403715523912143104.png   

/home/chenchen/Dataset/V1_02_medium/mav0/cam0/data/1403715523962142976.png





## Dmatch

queryIdx：测试图像的特征点描述符的下标（第几个特征点描述符），同时也是描述符对应特征点的下标。
trainIdx：样本图像的特征点描述符下标,同时也是描述符对应特征点的下标。
distance：代表这怡翠匹配的特征点描述符的欧式距离，数值越小也就说明俩个特征点越相近。

**match()实际上调用就是knnMatch()把返回k个的匹配包了一层皮，设置返回一个最匹配的点**

___________

# 有关相机

值得参考的博客

+ https://blog.csdn.net/cuglxw/article/details/77885572   纯双目的矫正
+ https://blog.csdn.net/heyijia0327/article/details/83583360   双目与imu的标定，，先标出双目相机的内外参数，再联合imu标定
+ https://blog.csdn.net/u011178262/article/details/83316968#_images__imu__201    kalibr 标定全过程的总结
+ https://zhuanlan.zhihu.com/p/80797583?from_voters_page=true   小觅相机的标定
+ https://blog.csdn.net/okasy/article/details/90665534#t7  相机模型及畸变模型的总结
+ https://blog.csdn.net/u011475210/article/details/79185543?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task   有关鱼眼相机的矫正
+ https://mp.weixin.qq.com/s?__biz=MzIxOTczOTM4NA==&mid=2247488433&idx=1&sn=456487683cd56ffa7ac2ed622c652d9a&chksm=97d7f626a0a07f303abacf7ac5b135de8da4455c4f50927c2f5d0798b162d8909cbf076aaf73&token=1349743125&lang=zh_CN#rd  mei相机模型

## 摄像头畸变

distortion_model: radial-tangential     Radtan 模型即 radial-tangential 模型，包括径向畸变参数和切向畸变参多项式畸变模型，

k1,k2 径向畸变，，  P1，P2 切向畸变  ——-————Pinhole 模型与MEI模型相机的畸变参数不同

##　双目相机的矫正

+ 相机各自的内参矩，畸变系数，把两相机摆正各自所需的旋转矩阵
+ 再用两相机的内参构建一个共同的理想内参矩阵（左右的参数取了个平均而已）![image-20200312111353281](/home/chenchen/.config/Typora/typora-user-images/image-20200312111353281.png)

　

+ 双目平行矫正的过程
  + 将左右图像用**理想的共同内参矩阵转换**到各自的相机坐标系
  + 旋转摆正（极线平行）——将相机坐标系下的点，按照相**应的旋转矩阵，旋转摆正**（绕固定轴旋转，**左乘**旋转矩阵）
  + 旋转后的点进行**去畸变操作**
  + 将相机坐标系下的点，按照各自的内参矩阵，**转换到各自的图像坐标系**
  + 像素插值

## 标定的注意

+ 角点重投影是为了显示一下用计算出来的相机矩阵对角点进行重投影得到的理论位置。也是为了显示，理论得到的角点与实际角点之间的差别。

+ b. 选择合适的相机模型；根据经验吧，小FOV（小于90度）的用pinhole+radtan模型，广角的用pinhole+equidistance模型，然后还有全向镜头的模型   小觅相机的fov较大

+ *畸变图像和imu之间的外参数 和 rectify 图像跟imu之间的外参数是不一样的。*

+ 通常用没有去畸变的图像与imu做外参标定，

+ ```
  pinhole-equi 模型，对畸变大的相机效果不错
  ```

+ pinhole（omni） + radtan（equidistant）相机模型    distortion_model: fov？？？？

+ **T_cn_cnm1//左右摄像头的相对位姿** （右目相机在左目系下的表示）——只是旋转右目相机，使得左右相机对齐，，，

+ 而在ORBslam的配置文件中，为了对齐，左右相机都有旋转——————opencv的模式

+ ### MEI Camera        Omni + Radtan

+ 畸变矩阵与内参矩阵是相对应的，——————相机的模型不一样，即使同一种畸变模型，畸变参数也不一样

+ 相机的去畸变，，从图像像素坐标转至归一化平面坐标，然后去畸变，，在转回到图像坐标

## mei相机模型

相机坐标系下的3d坐标点

![image-20200314212055487](/home/chenchen/.config/Typora/typora-user-images/image-20200314212055487.png)





________



​	**opencv中mat矩阵的操作　　__a.at<float>(1,0)**



