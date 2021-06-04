

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
+ https://blog.csdn.net/humanking7/article/details/45037239      畸变矫正图片的过程

## 摄像头畸变

**在opencv2.0及其以前的版本，仅有针孔相机模型着一种———**

**———OpenCV：cv: pinhole + Radtan   , cv::fisheye: pinhole + Equi ,  [cv::omnidir](http://man.hubwiz.com/docset/OpenCV.docset/Contents/Resources/Documents/dd/d12/tutorial_omnidir_calib_main.html): Omni + Radtan**

distortion_model: radial-tangential     Radtan 模型即 radial-tangential 模型，包括径向畸变参数和切向畸变参多项式畸变模型

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

## mei相机模型—— Omni + Radtan

相机坐标系下的3d坐标点

![image-20200314212055487](/home/chenchen/.config/Typora/typora-user-images/image-20200314212055487.png)



________________________

https://docs.opencv.org/3.4.1/db/d58/group__calib3d__fisheye.html   **opencv中，有关fisheye(pinhole + equi )模型**

https://docs.opencv.org/3.3.1/d3/ddc/group__ccalib.html#ga0d0b216ff9c9c2cee1ab9cc13cc20faa   **opencv中，有关MEI(omni + radtani )模型**

------

## 图像畸变矫正的解释

+ https://blog.csdn.net/humanking7/article/details/45037239      畸变矫正图片的过程

+ https://blog.csdn.net/guanguanboy/article/details/93976129?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase   图像畸变与去畸变

+ 相机内参矩阵，fx, fy, cx, cy 是相机的物理参数，归一化平面坐标到像素坐标的缩放与平移，与相机的焦距及一个放大倍数决定

+ 首先是畸变的产生过程

  + (x, y, 1)~(u, v)  是没有畸变时的归一化平面坐标及像素坐标

  + 根据14讲书上的原公式，相当于是从正常图像得到畸变后的图像，如果说畸变量是(dx, dy), 则在归一化平面上有了畸变的坐标为(x', y', 1) = (x+dx, y+dy, 1)——这是一个施加畸变的过程。对应的畸变后的图像坐标为(u', v')

  + 根据(u, v)~(u', v')正向畸变产生的过程，通过该函数 cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false)， 得到逆向的去畸变过程的映射图。

  + 比如，根据畸变的施加过程，得到(1, 1)像素坐标，经过畸变以后在(2, 3)。则我们便将已有的已经产生了畸变图像中的(2, 3) 放到 (1, 1)处，得到去畸变以后的图像坐标

  + remap函数，就是直接根据给定的map进行图像的重映射   

    ```cpp
    ptrX[j] =(float)(Cols - j);
    			ptrY[j] = (float) i;
    //这种就是图像左右翻转
    ```

    + 从畸变的像素位置得到去畸变后的像素位置是比较复杂的，这是一个逆向的过程，由于函数关系是单调的，一般采用迭代法，这样方便编程实现和计算机的计算





**从畸变了的图像平面上，得到矫正后的相机系下归一化的平面坐标 ——undistortPoints()**

+ 直接使用内参矩阵，将图像平面的点(u, v)转到归一化平面(x, y)， 计算畸变量（dx, dy）, 则矫正以后的点坐标为（               (x-dx, y-dy）——减去，是去畸变（事实上，这是一种近似操作）——也可以时迭代逼近 Recursive distortion model
+ MEI相机模型——从图像平面到归一化平面，直接去畸变操作，无须再考虑折射那个过程

```c++
            x_dist = (it.x - cx()) / fx();
            y_dist = (it.y - cy()) / fy();

            x_corr = x_dist;
            y_corr = y_dist;  //转到归一化平面
            
             r2 =x_corr*x_corr + y_corr*y_corr;
                r4 = r2 * r2;

                cdest_inv = 1 / (1.f + k1_*r2 + k2_*r4);
                a1 = 2.f * x_corr * y_corr;
                a2 = r2 + 2 * x_corr * x_corr;
                a3 = r2 + 2 * y_corr * y_corr;

                deltaX = p1_ * a1 + p2_ * a2;
                deltaY = p1_ * a3 + p2_ * a1;  //计算畸变量

                x_corr = (x_dist - deltaX) * cdest_inv; //去掉畸变量
                y_corr = (y_dist - deltaY) * cdest_inv;
        //按照正常，畸变量应该是由未畸变的vy计算得到的，但在这里，是使用畸变后的xy去计算畸变量
```



________

## Mat矩阵数据类型

### 	使用Mat.at< >( u, v)访问矩阵　　__a.at<float>(1,0)       _____.at<数据类型>(u, v)

+ Vec3f v3f = mat.at<Vec3f>(y, x);//read color values for pixel (y,x)     访问指定像素的色彩值
+ image.at<Vec3f>(u, v)[channel]     获取指定通道的颜色

### 使用Mat.ptr<>()[]访问矩阵————（）里先访问行，[ ]里是列——————-图像需要连续

+ unchar  *data = image.ptr<unchar>(3)[4]   表示访问的是3行的4个元素
+ unchar  *data = image.ptr<unchar>(3)       指向的是3行的首个元素

### 使用前必须先初始化大小

### 使用迭代器遍历Mat的每一个像素

```c++
cv::Mat image;
cv::Mat = cv::Mat::zeros(cv::Size(height, width), CV_32F);  //Mat的定义也是先行后列，同样，遍历访问的时候也是先行后列
cv::Mat_<cv::Vec3b>::iterator it;
for(it = image.begain<cv::Vec3b>(); it != image.end<cv::Vec3b>(); it++)
{

}
```





___________

## 图像的填充，连续？

Mat连续，指的是每个像素在内存中没有间隔的连续存储。但是，经过剪裁与填充的图像不再连续——这是因为新的剪裁的图像与原图像共用内存，新的图像是在原来的图像上取了一部分，所以可能不连续

###  将不连续的图像转换为连续的图像

```c++
if(!mat.isContinuous())
{
  mat = mat.clone();
}
```

____________

**cv::Size image_size(width_, height_);      image_size就是一个cv::Size 类型的数据**

**cv::Size(width_, height_);**

