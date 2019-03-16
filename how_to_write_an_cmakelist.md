我们写的C/C++程序需是要经过编译器处理, 最终变为二进制文件才能被计算机识别的. 一般我们程序生成的二进制target分为可执行程序和库文件. 可执行程序是我们接触得最多的(比如windows下的.exe), 只能执行, 并且更具自己代码所决定的流程一套完整的走下来. 库文件中包含了许多方法和函数, 可以被其他target调用(比如我们用opencv库中的函数来处理图像).

**gcc/g++** 是常用的编译器, 用来处理我们的c/c++程序并将其生成所需要的target. 假设我们写了一个`hello_world.c`, 并且想要将其生成可执行程序来运行,可以输入指令:
````
gcc -o hello hello_world.c
````
将会生成一个名为`hello`的可执行程序, 执行`./hello`就可以运行这个程序. 当代码文件较少时我们可以直接使用gcc生成target, 当文件较多时这样就很费时了. 于是就有了**make工具**来批处理文件, 调用gcc/g++来帮助我们生成target. 使用make工具需要编写规则文件**Makefile**(比如将哪些cpp文件生成可执行程序, 生成的target需要依赖哪些库等等), make工具将根据Makefile来批处理编译. 当项目工程比较大时, 直接编写Makefile还是比较复杂的, 于是就有了**cmake工具**来帮助我们自动生成Makefile. 使用cmake工具也需要写一个规则文件叫**CMakeList.txt**. 相对而言, 写CMakeList.txt是比较简单的了.

---
## cmake工程的结构
一个基于cmake的c++工程的典型结构为:
````
project_name        # 工程根目录
└── src             # 存放源码目录
    ├── xxx.cpp
└── include         # 存放头文件的目录
    ├── xxx.hpp
├── CMakeList.txt   
````
编译的方法为:
````
cd project_name         # 进入工程根目录
mkdir build & cd build  # 建立一个build目录并进入
cmake ..                # 执行CMakeList文件, 生成Makefile
make                    # 根据生成的Makefile编译生成target
make install            # 可选, 安装文件到电脑
````
---
## 如何利用CMakeList生成target
假设我们有一个叫做`basic_cmake_example`的工程, 其结构和上面的典型结构一样. 其中src目录下有一个hello.cpp的源文件, 内容为:
````
#include <iostream>
int main() {
  std::cout << "hellow world!\n";
}
````
我们想要将其生成一个可执行文件, 那么CMakeList.txt可以写为:
````
cmake_minimum_required(VERSION 3.0)

project(hello)

add_executable(hello_world   
  src/hello.cpp)
````
按照上面的编译方法编译后, 将会在build目录生成一个名为`hello_world`可执行文件, 执行`./hello_world`, 将会在终端输出`hello world`

在上面的CMakeList中, 第1行`cmake_minimum_required(VERSION 3.0)`是必不可少的. 这条语句指定了所需的cmake的最低版本要求, 也就是说我们电脑上装的cmake版本要比这个数字高才能编译这个工程.(cmake版本可以通过`cmake --version`查看.) 第2行`project(hello)`也是比不可少的, 设定了这个工程的名字, 名字可以任意取. 可以很容易猜出来, 我们通过第3行的指令`add_executable()`生成了一个名为hello_world的可执行文件. 简单的[add_executable](https://cmake.org/cmake/help/v3.3/command/add_executable.html)指令为:
````
add_executable(<target_name>   
               <source-file1> <source-file2> ...)
````
其中<target_name>是生成的可执行文件名字, 可以任意取. 后面的参数需要填生成这个可执行文件所需要的**所有源文件相对于CMakeList的路径**.

相应的, 当我们想生成库文件时可以用指令[add_library](https://cmake.org/cmake/help/v3.3/command/add_library.html):
````
add_library(<target_name> [STATIC | SHARED | MODULE]
            <source-file1> <source-file2> ...)
````
这个指令和add_executable类似. 当指定STATIC时, 将生成静态链接库；当指定SHARED时, 将生成动态链接库. 指令默认生成静态链接库.

上面的例子中只用一个源文件生成了可执行文件, 并且不依赖于其它库文件. 假设我们的程序要依赖OPENCV, 使用OPENCV中的函数来处理图片呢? OPENCV库文件(一系列.so文件)被安装到了电脑上的其它位置, 如何才能让我们的程序和OPENCV发生关系, 并能够调用其中的函数呢? 这时CMakeList可以写为:
````
cmake_minimum_required(VERSION 3.0)

project(hello)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(hello_world   
  src/hello.cpp)

target_link_libraries(hello_world
  ${OpenCV_LIBRARIES})
````
在上面的CMakeList中, 我们使用`find_package()`来寻找OPENCV, 一旦找到的话, 就会在变量`OpenCV_INCLUDE_DIRS` 中存储OPENCV的头文件所在路径, 在变量`OpenCV_LIBRARIES`中存储OPENCV库文件所在目录. 我们可以使用[message()](https://cmake.org/cmake/help/v3.5/command/message.html)指令来打印变量的值, 比如在`find_package`后面添加:
````
message("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
````
在执行`cmake ..`时, 会在终端中打印出上面两个变量存放的值. 如何用`find_package`找包请参看[how to find an cmake package](./how_to_find_an_cmake_package.md).

[include_directories()](https://cmake.org/cmake/help/git-master/command/include_directories.html)中填库头文件目录的绝对路径, 或者是相对于当前CMakeList.txt位置的相对路径, 这样我们在程序中就能直接使用相对路径包含头文件. 举个例子, 假设OpenCV的头文件目录为`/usr/local/include/opencv3.3.1/opencv/`, 如果不使用`include_directories()`, 我们在程序中包含opencv头文件的方式为:`#include </usr/local/include/opencv3.3.1/opencv/cv.hpp>`. 而使用了`include_directories()`后, 我们在程序中包含头文件的方式可以简单的写成:`#include <cv.hpp>`.

[target_link_libraries()](https://cmake.org/cmake/help/git-master/command/target_link_libraries.html)的作用是为target链接上所需要的库文件. 一般头文件中包含了函数的声明, 库文件中包含了函数的实现. 如果不链接到相应的库文件, 那么就无法调用函数的具体实现, 会报`undefined reference ...`错误. `target_link_libraries()`的第一个参数为target名字, 应该要与`add_executable`和 `add_library`中的名字一致；后面的参数为所需库文件的绝对路径.

以上就是一个基本的CMakeList.txt文件的写法. [simple_cmake_example](./simple_cmake_example/)提供了一个简单的例程. 例程中生成了一个叫`point`库和一个叫`simple_cmake_example`的可执行文件. 可执行程序中调用了opencv库显示一张图片.

---
## 如何写一个基于ROS的CMakeList
一个ros工程目录的基本结构为:
````
workspace_name   # 工作空间目录
    └── devel    # 编译后自动生成该目录. 生成的target存放在该目录
    └── src   # 源码目录
        └── package1    # 包目录
            └── src     # 存放源码目录
                ├── xxx.cpp
            └── include    # 存放头文件的目录
                ├── xxx.hpp
            ├── CMakeList.txt   
            ├── package.xml  
        └── package2    # 包目录
         ...
        └── packagen    # 包目录
````
可以看到, 一个ros package就是上面介绍的一个cmake c++工程, 只不过多了一个`package.xml`文件. 另外, 一个ros package必须放在workspace目录下的src里才行.

编译ros包的基本指令:
````
cd <workspace_folder> # 进入工作空间目录
catkin build <package-name>
````
更多关于ros的基本概念请先参看[ros官方教程](http://wiki.ros.org/ROS/Tutorials).

ROS package的CMakeList与普通CMakeList的写法基本是一样的, 普通CMakeList支持的语法, ros CMakeList都支持. 只不过ros对cmake进行了封装, 增加了几条指令. 和普通的CMakeList相比, 这里主要关心2个问题: 1) 如何找到其他的ros package作为库使用(找普通的library方法不变). 2) 如何让自己写的ros package能够被其他ros package找到使用.
### 如何使用其他的ros包
寻找ros包同样也使用`find_package()`指令, 不过有些许不同:
````
find_package(catkin REQUIRED COMPONENTS
  <package1>
  <package2>
  ...
  <packagen>)
````
可以看到, 就算有n个ROS包, 也可以使用1个`find_package()`来找. 所有的ROS包都将作为catkin的components, 这些包的头文件存储在变量`catkin_INCLUDE_DIRS`中, 库文件都存储在变量`catkin_LIBRARIES`中. 找ROS包除了在`CMakeList.txt`中使用`find_package`, 还需要在`package.xml`文件中添加:
````
<depend>package1<depend>
...
<depend>packagen<depend>
````
假设工作空间下已经有一个`package-A`. 现在我想写一个`package-B`, 需要使用`package-A`中的函数. 此时需要在`package-A`的CMakeList.txt中添加:
````
find_package(catkin REQUIRED package-A)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(<target_name>
  xxx.cpp ...)
target_link_libraries(<target-name>
  ${catkin_LIBRARIES})
````
然后在`package-B`的`package.xml`文件中写入:
````
<depend>package-A<depend>
````
这样就能在package-B的代码中包含package-A的头文件并使用其中的函数了.

**ROS包一定是位于某个工作空间中的(可以是其他工作空间), 每一个工作空间都有一个setup.bash文件, 要想这个工作空间中的包能被`find_package()`找到, 必须先在终端执行 `source setup.bash` 命令来设定相应的CMAKE PATH变量**

### 如何让自己的ROS包能被其他包调用
想要让自己写的ros包能被其他ros包顺利调用, 需要在**生成target的指令之前**添加:
````
catkin_package(
   INCLUDE_DIRS <自己包的头文件所在相对路径(相对于CMakeList.txt)>
   LIBRARIES    <自己包会生成的库的名字>
   CATKIN_DEPENDS <自己包所依赖的其他ros包的名字>
   DEPENDS <自己包所依赖的其他非ROS库的名字>)
````
* 如果`INCLUDE_DIRS`不填, 则其他ros包无法找到这个包的头文件；
* 如果`LIBRARIES`不填, 则其他包会找不到这个包生成的库文件, 会出现`undefined reference error: ...`;
*  `CATKIN_DEPENDS/DEPENDS`的作用在于: 当其他包调用这个包时, 不需要再用`find_package()`再去寻找一遍相同的依赖库. 举个例子, 假如我们自己写的packae_A中依赖了OpenCV, 如果在`catkin_package()`中写了`DEPENDS OpenCV`, 那么在其他包中使用`find_package`找 package_A时, 会自动加入OpenCV库的依赖, 而不需要再使用`find_package(OpenCV REQUIRED)`寻找OpenCV.

更详细的关于ROS CMakeList的知识, 参考[官网ROS CMakeList](http://wiki.ros.org/catkin/CMakeLists.txt).

例程[simple_ros_cmake_example](./catkin_ws/src/simple_ros_cmake_example)中展示了一个基本的ROS 版CMakeList写法．这个包读取一张图片并发布成占据栅格在rviz中显示，同时订阅rviz发布的`2D Nav Goal`信息．
