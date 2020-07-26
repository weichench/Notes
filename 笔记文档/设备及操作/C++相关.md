## 以linefeature_tracker为例，说明类的使用

+ 实例化，，东西已经有了，但里边是空的，，需要后续的操作在里边放东西

+ 主函数文件为使用（调用），其他.cpp与.h文件为定义

+ 在linefeature_tracker.h文件中，首先定义了帧类的类型**FrameLines**，然后定义了**LineFeatureTracker**跟踪器类的类型，LineFeatureTracker跟踪器类中，  **FrameLinesPtr  curframe_  forwframe_**     实例化了帧类型的成员，

+ 在linefeature_tracker.cpp文件中，为linefeature_tracker.h中声明的函数的具体定义，包括相关类的成员函数

  + **void LineFeatureTracker::readImage(const cv::Mat &_img,const cv::Mat &_img1)**，，就是LineFeatureTracker跟踪器类成员函数的一个具体定义（**LineFeatureTracker::**就表示是类的成员函数），，，成员函数即为对类的成员变量的操作，即在成员函数内，会对实例化的成员变量curframe_  forwframe_  进行操作

+ 在主程序文件linefeature_tracker_node.cpp中，**LineFeatureTracker trackerData;** 会实例化跟踪器类型的类。实例化后，便可对该类的成员变量及成员函数进行调用及引用

  +  **trackerData.readImage(image0,image1);**  调用类成员函数  ，可能会在实例化的类里放东西，
  +  **auto &ids = trackerData.curframe_->lineID;**  引用类成员变量

  **其他库文件似乎也可调用ros的函数？？？**
  
  
  
  + 在A类中，B类是A类的一个成员，但是A类与B类都想使用一个定义在C文件下的库函数，则C,h应该包含在A和B的.cpp文件中，而不能包在其.h文件中
  
  
  
  + .h文件中定义的函数，如果在相应的.cpp文件中没有定义的话，是会编译出错的
  
  __________________
  
  

## namespace 的用法

**using namespace std;**
**using namespace cv;** 

文件前加入该声明，则后续使用cout,,opencv的成员时，不用　**cv::mat**   或**std::cout**, 可直接用

.h文件中定义了命名空间，对应的包含了该.h文件的.cpp文件，可以直接使用该命名空间

__________________



+ 赋值语句的返回值为所赋的值
+ while (N <= (MSS[0] = rand() / (RAND_MAX/(N-1))));　　　可以产生小于Ｎ的随机数

________



### 强制输出

	printf("(INFINITE)");
			fflush(stdout);  //用于printf()函数后，防止连续使用该显示函数时出错

_______________________



### 自动遍历容器里的元素

   **for (auto &it_per_frame : it_per_id.linefeature_per_frame)**

it_per_frame直接指该成员   ，**引用**

**it_per_frame.lineobs**

**vector 容器**

**.begain()  与 .end  返回当前容器的迭代器**  ->

**.front()   .back()  返回当前容器的引用  **  .

**栈只有.top()    而队列有 .front() 与 .back();**

## 删除vector中指定元素——迭代器，erase( )后，指向下一个元素

## 对于set，也可使用同样的操作，只是set在添加元素用的insert( )函数

```c++
 for(vector<int>::iterator iter = array.begin(); iter != array.end();)
	 {
	   if(*iter == 9)
	     iter = array.erase(iter);  //删除后，返回下一个的指针
	    else
	      ++iter;
	 }

vector<int>(numLines, 0);      //初始化数组的长度及初值
```







________________________



##  嵌套vector的赋值————逐层push_back()

 ``vector<vector<int>> abc;`
  `vector<int> abc_;
  abc_.push_back(25);`
 `abc.push_back(abc_);`
 cout<<"*****test is "<<abc[0][0]<<endl;`

________

## 多线程

1. **std::thread t(func)**  ，创建并行的线程，并执行
2. **t.join()**，可放在线程创建后合适的地方，其会**等待**被创建的线程执行结束，并**回收执行线程的资源**，比如相关的变量。随后，主线程才会继续进行，，，若主线程没有执行到**t.join()**，资源未回收，资源泄露
3. **t.detach()** ，**直接放在线程创建之后**，将被创建的线程与主线**程彻底分开**，资源数据不能通信，，即使主线程死掉，被创建的线程也无影响，继续执行，，**相当于分成了两个完全独立的程序**
4. 并行提取双目线特征，会用到**.join()**函数，，用于在等待并行线程结束后，获取线程的资源（执行结果）

_____

## 互斥锁与线程协同

**std::mutex m_buf;**

​     m_buf.lock();
​    img0_buf.push(img_msg);
​    m_buf.unlock();

**上锁与解锁需成对出现**

+ 对于全局变量（共享资源），线程A在使用全局变量时，会加锁
+ 线程B同时也想加锁访问全局变量时，则线程阻塞，等待线程A解锁后，线程B会被唤醒，才去加锁访问全局变量
+ 若线程C直接访全局变量（共享资源），则可访问，但数据很可能会乱掉

**互斥锁，当并行的多个线程对全局变量（共享资源）加锁访问时，线程的阻塞可实现并行线之间的协同工作**

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

**用于将当前线程休眠2ms，留出些时间，以防该线程一直占着共享资源??**

## **并行线程与锁也可以成为类的成员**

__________

### ros订阅的话题

**n.subscribe("/feature_detect/feature", 2000, feature_callback)**

**“/ 节点名称 / 该节点发送消息的话题名称”**

## rosbag play

**-l 代表循环**播放

 **-r 0.5 代表以0.5倍速**度播放

 **-s 10 代表从第10秒**播放，

**-d 2 表示2秒以**后再播bag文件，默认0.2秒后播放

+ **ros::spin();**  ROS消息回调处理函数，，，spin期间，执行回调函数，否则，接受到的消息存放在buffer中，并未被回调函数处理
+ **有关rosbag的介绍**   https://blog.csdn.net/orange_littlegirl/article/details/93128037    

_______________

## extern int a； 声明定义的为外部变量

头文件是用来声明的，如果是变量，在头文件，一般都是声明的外部变量，比如**extern int a** (不可以赋值，实力化，仅仅是声明)，但还需要在相应的.cpp文件中，进行定义（实例化），**int a = 1**，  这样以后，才可以在其他地方调用该外部变量





__________

**Vector3d(vps[i].x, vps[i].y, vps[i].z)**;  std类型向量的构造

______________

## RVIZ中可视化

https://blog.csdn.net/u013834525/article/details/80447931



______

## sprintf()函数—  

可能导致缓存器溢出，使用snprintf(), 规定好要写入的字符长度，避免溢出

 snprintf(write_pt2, sizeof(write_pt2), "%.3f %.3f", last_observe.lineobs[2],last_observe.lineobs[3]);

_______

## 程序内，好多都是模仿的，未知其原理，比如  .   或  ->   的使用区别

_______

## vector.swap( )函数

__________

## map

 **map<double, ImageFrame>::iterator it_0;**

  **it_0 = all_image_frame.find(t_0);**——————map的迭代器





**利用map的迭代器，删除其中的一个映射，同时考虑到了释放动态内存——指针指向的对象，考虑内存泄露**

 map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                **delete it_0->second.pre_integration;**
                all_image_frame.erase(all_image_frame.begin(), it_0);

 



根据索引，在map中查找

https://www.cnblogs.com/tp-16b/p/9156810.html    介绍map的文章

**parameter_block_size[it.first]**         先得到map的key值，再取得相应的value值, const修饰的map，不能用[ ]获取value

**const 修饰的map，不能根据key去得到value值？  ——验证，是真的不行**

对于整个的map，是没有second这个成员的，只能根据key值获取value值，或者通过迭代器去遍历。map相当于是个映射集，而对于其中的一条映射，其first就是key值，second就是value值。**注意看清对象是整个容器，还是容器内的一个成员，**

map实际上，也就是一些pair的集合，因为pair时有first与second这个操作的

**map.count(key) == 1  判断map是否有某个key值**

**map[key] = n, map是可以根据key值对value值进行更改的**





__________

## list 的遍历方法

  **for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)**
    **{**
        **it_next++;**

**linefeature.erase(it);**    删除掉列表中的元素

**continue** ;  如果这样删除的话，后边要跟continue

**}**

____________

#include <iostream>

使用cout时，需要包含的头文件



_________

## 仿函数

bool operator( )    把类当成函数来使用

_______

## erase(it)函数

**要注意，执行这个函数，删除了it指向的内容后，it指针没动，但其已经指向的是下一个元素，这时，如果继续指针前移，继续下一个循环，则会漏掉一个元素，所以常规操作是让指针先it--，后退一位，再去it++ 下一个循环**

______

**sizeof(data)/sizeof(int)**  判断出数组的长度

**数组名，常量指针**，系统会知道其长度，所以上面的sizeof(data),是整个数组的字节长度————但是，如果将数组名作为函数的形参传递，其相当于复制了一个指针（在内容上，还是实参，不是复制的），其退化为一个普通的指针，sizeof(data1)，就仅仅只是一个指针的字节长度。

**int data[ ],   data就是一个指向数组的指针**

**sizeof()  得到的是字节数**

int arr[] = {0}    只是一个为0元素的数组，并非不定长数组

{  } 限制了所定义变量的作用域

https://blog.csdn.net/zhanshen112/article/details/80758850   动态数组

## C++ 二维数组

**std::vector<std::vector<int>> ind_cs;**

  **for(int i=0;i<ind_cs.size();i++)**

**{先是遍历行，再是每行的列}**



_______

## nullptr 空指针

______

## 动态大小的矩阵      ,运行时确定大小

**Eigen::MatrixXd A(pos, pos);**
**Eigen::VectorXd b(pos);**

______

**变量名与函数名不能重复**

______

## 虚函数

生成虚函数表，在类型中生成一个指向虚函数标的指针，64位机器，指针占8个字节，意味着：虚函数表是没有放在类的里边的，只是放了一个指向该虚函数表的指针，**对比析构函数与构造函数**



_____

## 析构函数

**类成员变量为指针，都是会在动态内存空间开辟一块内存区域，然后该指针变量指向这块区域, 然后再在这块动态内存中存东西**

类的成员为指针变量，当其指向动态内存里一部分内存空间时，调用delete销毁该类的时候，析构函数自动调用，释放其指向的那部分动态内存空间

+ 构造函数与析构函数的地址是不能被获取的，不能形成指向他们的指针，理解为：类型里边并不会存在这两个函数的地址，**但是对于普通的成员函数，类里边应该是村存储这个函数的地址？？**

____________

## 赋值运算符函数  operator = 

https://blog.csdn.net/liitdar/article/details/80656156

- 赋值操作函数应该返回该类自己本身的引用，**返回值*this**, 如此，便可以连续赋值
- 传入的应该为实参——常量引用  **const class& class1;**
- 这个类指针类型的成员变量可能已经占用一块动态内存空间，则应将其释放，否则**内存泄露**
- 需要判断传入的实参是**否为其本身**，若是，则直接返回 *this , 因为一旦是其本身，释放动态内存空间，则会出问题

**当新开辟一块动态内存空间时，万一内存已经不足，则可能会出问题**

________________



## 复制构造函数

##  复制，即clone出一个新的————赋值，对一个已经存在的进行赋值，也所以，在赋值时，要释放掉其本已指向的动态内存空间（delete以后，随即让该指针变量 = nullptr）



当类构造函数传入的参数，也是该类型的一个类，此即为**复制构造函数**

函数的传入参数类型是形参，则会复制一个参数。当复制构造函数的参数也是形参，则在复制这个类的时候，又会调用这个复制构造函数，从而形成递归调用，无限循环———因此，复制构造函数只允许**实参传递——A(const A& other )**

**const 会跟离他最近的先结合**，在这里，const A  ———意味着将函数内A类型的实参other视为一个常量——指向常量的指针

A  *const other ——先跟指针结合，表示一个常量指针，相当于引用

___

## vector

当超出长度，以2倍扩容——带来额外操作

使用STL库，在cmake文件中要添加**set(CMAKE_CXX_FLAGS "-std=c++11")**，表示引入c++11的特性

### 通过迭代器遍历vector——方便删除其中的某一个元素



______

## 字符串

strcpy(地址/指针/数组名，字符串)  ——————将字符串复制到目标地址

指针2 = strdup（指针1）    将指针1指向的内容复制一个放到动态内存空间，让后让指针2指向这块动态内存空间

if(数组1 == 数组2)  事实上判断的是：是否为同一个地址的数组

完全相同的常量字符串在内存（栈内存）中只有一个拷贝



## 静态内存，栈内存，（堆）动态内存——时刻注意内存泄露

char *p = "dasasdg";   char *q = p ;   对于在栈内存中，即使对于q时不可以在整个字符串后边在家的，**字符串指针是不能超出范围的**———— 所以，只能提前指定一块大小的区域，char q[20]，或者在动态内存区域 q = new char[n](动态内存区域的好处是，可以在程序运行过程中再确定大小（动态内存空间的好处是可以在程序运行过程中再决定其大小） 

除非说使用vector，否则是不可能随意在后边继续添加内容的——因为可能超出范围

**指针指向数组名，就是指针指向数组的第一个成员，由此，这个指针便可以操作整个数组**

在动态内存区域，int *p = new int[10]， **则不用初始化，这个动态内存数组里的元素初值都为0**。 但是，普通的栈内存，定义了数组后，初始化以前，里边的元素是乱的。**所以，栈内存定义的变量，都要注意初始化。**



_________



## 有关指针的移动

指针的移动必须是在一个确定的区域内的，比如一个数组内。对于一个常量字符串，是绝对不能超出这个范围的。如果要对一个常量字符串扩充长度，那只能复制出来，用一个更大的内存空间（一个更大的数组，后者在动态内存空间开一个程序运行时才确定大小的数组）先存储，再往里边添加新的内容。

**当指针p指向数组的时候，p[i]也是可以的**

## 指针p指向数组或者字符串

**则 *(p+i)       p[i]             p=p+i; *p     这几种获取指向值的操作方式是等价的**

**指向二维数组的指针p，则获取i行，j列元素的操作是 *(p + i * cols + j )  或者 p[ i * cols + j ]** 



_______

## 栈

.top( )返回栈顶元素   。pop（）出栈

_____________

## 随机数

rand()伪随机数，如果没有给出种子，每次运行时，如果没有给出种子，每次运行得到的都是相同的值

```
seed = time(0);
srand(seed);

rand_number = (rand()%(99-0+1) + 0);
```



__________

## 迭代器——就是一个指向容器元素的指针——跟指针的用法一样



___________

###  pow(x,y)   返回值为x的y次方

__________

## 一个文件中，定义在函数体外的变量为全局变量

__________

## 链表——动态扩展，不用闲置内存，空间效率高

## 函数传入链表的头指针，务必先判断是否为空指针

最尾的节点指针指向nullptr，用于找到表尾

**链表的表头本身就是一个指针，如果实参传递操作这个指针本身，则需要指向指针的指针 ****phead., 在函数内，其本身为*phead   。否则，只是传入形参 *phead, 则相当与在函数内部复制了一个指针，除了函数，啥都没有**

**当链表的表头可能会被改动时，就应该实参传递，传递表头的地址**

**任给一个链表中的节点，就要考虑该节点是否为表头或表尾**

新生成一个表头指针，将表头作为实参传递给函数时，必须给表头先赋一个节点，否则，出错

```c++
void add(listnode **phead, int value)
{
  if( *phead == nullptr ) ; //会运行出错    
      if((*phead)->value == value);  //->的优先级高于*, 所以必须要给(*phead)加括号
   if(*phead != nullptr )
   {
       if((*phead)->next == nullptr || (*phead)->value == 5 );
       //只有在先判断该指针不为空的前提下，才可以去访问节点的成员
   }
}




```



**根据value删除链表的一个节点时，要考虑到表头，表尾，还有 value并不存在于链表中这些特殊情况**——可以根据值，也可以根据指针去删除一个节点



**在链表中，要让两个指针一前一后错开地往前遍历，则遍历前就要让这两个指针错开，然后往前遍历**

**但凡通过指针访问链表的成员，都得考虑该指针是否为一个空指针**



## 传入的参数为链表表头指针的函数——主要参考JZ18

1. 首先判断表头是否为空，以及考虑是否仅有一个节点的情况
2. 考虑要处理的情况在是否可能在表头
3.  时刻留意，在所有的操作后，指针是否可能为空，一旦空掉，如何处理
4. 遍历访问每个成员，首先判断指针是否为空
5. 时刻留意，是否已经到了表尾



_____

## 函数名作为形参传入——传的是函数名的指针——提高代码的可扩展性

```c++
void function(int a, bool (*func)(int))
{
   int b = func(a);
}
```



____________

可以在头文件的函数声明中，对形参赋予默认的参数

```c++
//在头文件中的声明
cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

//在.cpp文件中的具体定义
cv::Mat
PinholeCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                       float fx, float fy,
                                       cv::Size imageSize,
                                       float cx, float cy,
                                       cv::Mat rmat) const
{
    
}


camera->initUndistortRectifyMap(map1,map2);  //调用时，有默认参数的形参可以不出现

```

_______

**θ = atan(y / x)求出的θ取值范围是[-PI/2, PI/2]。**

**θ = atan2(y, x)求出的θ取值范围是[-PI, PI]。**

_________

**cmake 可执行的程序名称不能为test**

________________

## 参数参数为引用

```c++
void function(int &a, int *b)  //定义的时候为，参数为引用
{}

int b= 5;
int a=5;
int *p = &b;
function(a, &b);  //  就是这样调用的   而不是 function(&a) 
function(a, p);
```

**指针只能指向基本的类型，比如int*,   或者一个结构体的指针， 或者一个类的指针**

**但引用可以是任何数据类型，比如vector，map，pair，任何数据结构都可以（但没见过对一个类的引用），所以通常在函数传递实参的时候，数据（数据结构，容器）使用引用传递，类使用指针**

