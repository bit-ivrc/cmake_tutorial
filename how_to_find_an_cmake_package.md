# 如何使用 find_package 找包
这里只是针对Ubuntu下的应用做了一个简单的整理, 详细的教程可以参考[官方find_package教程](https://cmake.org/cmake/help/v3.5/command/find_package.html)和[Find Module教程](https://cmake.org/cmake/help/v3.3/manual/cmake-developer.7.html#find-modules)

---
## 一. find_package 的工作方式
cmake下找包分为两种模式: **Config-file Mode** 和 **Find-module Mode**. **Config Mode**需要寻找`<package>Config.cmake`或`<lower-case-package>-config.cmake`文件, 而**Module Mode** 需要寻找`Find<package>.cmake`文件. 无论哪种模式, 找到package后一般都会对以下cmake变量赋值:
````
<package>_FOUND: 找到为true, 否则为false
<package>_INCLUDE_DIRS: 库头文件所在路径
<package>_LIBRARIES/<package_LIBRARY_DIRS>: 库文件所在路径
<package>_VERSION/<package>_VERSION_STRING: 库的版本号(不一定有值,除非文件中提供了版本信息)
````
简单的找包指令为:
````
find_package(<package> [version] [EXACT] [QUIET]|[REQUIRED]
              [[COMPONENTS] [components...]] [CONFIG]|[MODULE])
````
其中 `<package>`为包的名字;`[version]`为版本号, 可以不填. `[EXACT]`可以不填, 当使能时, 要求找到的库版本号和要求的完全一致才算找到. `[QUIET]`和`[REQUIRED]`填一个就行, 使能`[QUIET]`时, 就算找不到库, cmake也不会报错会继续执行; 使能`[REQUIRED]`时要求必须找到库, 找不到时会报错退出. 使能`CONIFG`选项时, 使用Config模式找包; 使能`MODULE`选项时, 使用Module模式找包. **当没有指明具体用哪种模式时, `find_package`会先使用`Module Mode`找包, 如果找不到再使用`Config Mode`寻找**

### Config Mode 方式
`Config-file Package`是cmake支持的标准包, 一般通过源码编译安装的包都在`CMakeList`中使用cmake指令生成了相应的配置文件`<package>Config.cmake/<package>-config.cmake`和版本文件`<config-file>Version.cmake/<config-file>-version.cmake`(可以参考[how to install an config-package](how_to_install_an_config-package.md)). 因此通过源码编译安装的包一般都能使用Config模式找到. 因为在安装时指定了安装路径, 因此配置文件中直接就设置了库路径相关的变量, 也就是说找到了配置文件就找到了库. 另外, 版本文件中存储了这个库的版本信息用来和指定的版本比较.
使用Config模式找包可以设置的选项有:
````
find_package(<package> [version] [EXACT] [QUIET]
             [REQUIRED] [[COMPONENTS] [components...]]
             [CONFIG]   
             [NAMES name1 [name2 ...]]
             [CONFIGS config1 [config2 ...]]
             [HINTS path1 [path2 ... ]]
             [PATHS path1 [path2 ... ]])
````
* 当`[NAMES]`不填时, 默认寻找`<package>Config.cmake`, 否则寻找`<name>Config.cmake`. `<package>`或`name`都是**大小写敏感**的, 名字写错了, 可能就找不到包了.
* `[CONFIS] config1.cmake config2.cmake ...`, 这一项可以直接设定需要找的配置文件的名字, 当设置了这一项时, find_package只会去找这一项指定名字的配置文件.
* `[HINTS]`和`[PATHS]` 选项后面可以填猜测的配置文件所在路径

Config模式下, find_package会在以下路径中去寻找配置文件:
````
<prefix>/(lib/<arch>|lib|share)/cmake/<name>*/          (U)
<prefix>/(lib/<arch>|lib|share)/<name>*/                (U)
<prefix>/(lib/<arch>|lib|share)/<name>*/(cmake|CMake)/  (U)
<prefix>/                                               (W)
<prefix>/(cmake|CMake)/                                 (W)
<prefix>/<name>*/                                       (W)
<prefix>/<name>*/(cmake|CMake)/                         (W)
<prefix>/<name>.framework/Resources/                    (A)
<prefix>/<name>.framework/Resources/CMake/              (A)
<prefix>/<name>.framework/Versions/*/Resources/         (A)
<prefix>/<name>.framework/Versions/*/Resources/CMake/   (A)
<prefix>/<name>.app/Contents/Resources/                 (A)
<prefix>/<name>.app/Contents/Resources/CMake/           (A)
````
其中的(W)代表Windows系统, (U)代表Unix类的系统, (A)代表Apple系统. `<arch>`是由变量`CMAKE_LIBRARY_ARCHITECTURE`指定的路径(在我的64位系统上默认为`x86_64-linux-gnu`). `<prefix>`为路径前缀, Unix类的系统下, 提供`<prefix>`的变量有6个:
* cmake cache变量: `CMAKE_PREFIX_PATH`
* cmake cache变量: `CMAKE_SYSTEM_PREFIX_PATH`
* 环境变量: `CMAKE_PREFIX_PATH`
* 环境变量: `PATH` 其中以`/bin`,`/sbin`结尾的路径会自动退回到上一级路径
* 由指令中 `HINTS` 指定的路径
* 由指令中 `PATHS` 指定的路径
(注: 环境变量就是系统shell中的变量; cmake cache变量可以理解成cmake中的全局变量, 是被存储在文件中的, 当执行`cmake ..` 后, 会生成一个`CMakeCache.txt`的文件, 里面记录了cache变量的值. 另外cmake还有普通变量, cache变量和普通变量的区别可以参考[cmake 两种变量原理](https://www.cnblogs.com/ncuneugcj/p/9756324.html)).

我们以找opencv为例, 看一下opencv是如何被找到的. 在`CMakeList.txt`中写入:
````
find_package(OpenCV 3 REQUIRED)
message("opencv config file path: " ${OpenCV_CONFIG})
message("1. cmake cache var 'CMAKE_PREFIX_PATH' = ${CMAKE_PREFIX_PATH}" )
message("2. env var 'CMAKE_PREFIX_PATH' = $ENV{CMAKE_PREFIX_PATH}" )
message("3. env var 'PATH' = $ENV{PATH}" )
message("4. cache var 'CMAKE_SYSTEM_PREFIX_PATH' = ${CMAKE_SYSTEM_PREFIX_PATH}" )
````
执行`cmake ..`后, 可以看到终端中有输出(每个人可能不一样):
````
opencv config file path: /opt/ros/kinetic/share/OpenCV-3.3.1-dev/OpenCVConfig.cmake
1. cmake cache var 'CMAKE_PREFIX_PATH' =
2. env var 'CMAKE_PREFIX_PATH' = /home/yangt/workspace/ros_ws/cw_project/ivrc_ws/devel:/opt/ros/kinetic
3. env var 'PATH' = /opt/ros/kinetic/bin:/home/yangt/bin:/home/yangt/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/usr/local/texlive/2018/bin/x86_64-linux:/snap/bin:/usr/local/texlive/2018/bin/x86_64-linux
4. cache var 'CMAKE_SYSTEM_PREFIX_PATH' = /usr/local;/usr;/;/usr;/usr/local
````
可以看到, 找到的Opencv为`/opt/ros/kinetic`下的3.3.1版本. 而能够提供这个路径`<prefix>`的变量有: 环境变量`CMAKE_PREFIX_PATH`和环境变量`PATH`. 如果我们在CMakeList中把这两个环境变量的值设为空, 那么将不会找到符合版本的Opencv. 在`find_package`前写:
````
set(ENV{CMAKE_PREFIX_PATH} "")
set(ENV{PATH} "")
````
再次执行`cmake ..`后输出为:
````
CMake Error at CMakeLists.txt:9 (find_package):
  Could not find a configuration file for package "OpenCV" that is compatible
  with requested version "3".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake, version: 4.0.0
    /usr/share/OpenCV/OpenCVConfig.cmake, version: 2.4.9.1
````
可以看到, 找到的opencv为`/usr/local`下的4.0.0和`/usr`下的2.4.9, 但由于版本不符合`REQUIRED`要求, 还是报错退出了. 如果继续把`CMAKE_SYSTEM_PREFIX_PATH`设为空, 将找不到任何版本的opencv.

### Module Mode
**Module Mode** 会去变量`CMAKE_MODULE_PATH`存放的路径下寻找名为`Find<package>.cmake`的文件. `Find<package>.cmake`是需要我们自己写的, 其中可以强硬的指定库路径所在位置, 也可以通过cmake提供的函数来自动寻找, 第二种方式更为鲁棒一些. `CMEK_MODULE_PATH`默认也是空的, 需要我们手动提供find-module文件的路径. 简单来说Module Mode就是允许让用户用自己的方式来找库, 这样更能确保找到自己想要的库. 像ceres, gtsam等这些库就是使用module模式来找自己的依赖库.

使用Module模式的话, 一般在工程目录下新建一个cmake的目录, 并在cmake目录中存放自己写的`Find<package>.cmake`文件:
````
project_name        # 工程根目录
└── src             # 存放源码目录
    ├── xxx.cpp ...
└── include         # 存放头文件的目录
    ├── xxx.hpp ...
└── cmake          # 存放Module文件的目录
    ├── Find<package1>.cmake ...
├── CMakeList.txt   
````
然后还需要在`CMakeList.txt`的`find_package`前加入find-module文件的路径:
````
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
````
#### 1. 如何写`Find<package>.cmake`文件
`Find<package>.cmake`文件的写法有很多, 十分自由. 这里提供一种简单的思路: 先尝试使用[pkg-config](https://en.wikipedia.org/wiki/Pkg-config)工具来找包; 然后将`pkg-config`的寻找结果作为hints, 使用`find_path()`找头文件路径, 使用`find_library()`来找库文件路径; 最后使用`find_package_handle_standard_args()`来检查设置相应的变量.
##### a) pkg-config方式
`pkg-config`是一个找库的工具. 一般通过.deb安装或者直接安装二进制文件库都会提供一个`.pc`文件, `.pc`文件中存放了库的相关信息. `pkg-config`就是通过寻找`.pc`文件来找到库的. Ubuntu下`pkg-config`会在以下目录里寻找`<package>.pc`文件(以下内容可以在终端输入`man pkg-config`看到):
````
1. /usr/lib/pkgconfig
2. /usr/share/pkgconfig
3. /usr/local/lib/pkgconfig
4. /usr/local/share/pkgconfig
5. 环境变量 PKG_CONFIG_PATH 存储的路径
````
也就是说**只有在以上路径中存放了`.pc`文件的库才能使用`pkg-config`工具找到.**
cmake中使用`pkg-config`工具的方法是:
* 先在`Find<package>.cmake`使用`find_package(PkgConfig)`找到`pkg-config`工具
* 然后调用`pkg_check_modules()`函数使用`pkg-config`来找包

`pkg_check_modules()`函数的简单用法为(详细的说明看[官方FindPkgConfig文档](https://cmake.org/cmake/help/v3.5/module/FindPkgConfig.html)):
````
pkg_check_modules(<PREFIX> [REQUIRED] [QUIET]
                  <MODULE>)
````
第一个参数`<PREFIX>`影响函数输出的结果, `pkg-config`找到库的话会设置以下变量:
````
1. <PREFIX>_FOUND    # 找到MODULE的话为true, 否则为false
2. <PREFIX>_LIBRARY_DIR    # MODULE所包含库的路径
3. <PREFIX>_LIBRARIES    # MODULE所包含的库的名字
4. <PREFIX>_INCLUDE_DIRS    # MODULE所包含头文件所在路径
5. <PREFIX>_VERSION    # MODULE的版本
````
第二个参数 `[REQUIRED]|[QUIET]`同`find_package()`中的一样, 前者找不到的话会报错退出, 后者不会.
第三个参数`<MODULE>`的形式可以是以下几种:
````
{MODNAME}            # matches any version
{MODNAME}>={VERSION} # at least version <VERSION> is required
{MODNAME}={VERSION}  # exactly version <VERSION> is required
{MODNAME}<={VERSION} # modules must not be newer than <VERSION>
````
需要注意的是`{MODNAME}`是大小写敏感的, **实际 `pkg-config`就会去找`{MODNAME}.pc`文件.**

##### b). 使用`find_path`和`find_library`
`pkg-config`只能找提供了`.pc`文件的库. 当没有`.pc`文件时, 我们应该继续尝试使用`find_path`和`find_library`寻找.
`find_path`指令为:
````
find_path (
          <VAR>
          name | NAMES name1 [name2 ...]
          [HINTS path1 [path2 ... ENV var]]
          [PATHS path1 [path2 ... ENV var]]
          [PATH_SUFFIXES suffix1 [suffix2 ...]]
         )
````
`find_path`的作用是寻找包含名为`name`的文件的路径, 并将路径存放到`<VAR>`中. 如果没有找到, `<VAR>`会被赋为`<VAR>-NOTFOUND`. Ubuntu下, `find_path`会去以下路径中寻找:
````
1. <prefix>/include 和 <prefix>/include/<arch>. 其中提供<prefix>的变量有: cmake cache变量'CMAKE_PREFIX_PATH', 'CMAKE_SYSTEM_PREFIX_PATH'和系统环境变量'CMAKE_PREFIX_PATH', 'PATH'
2. cmake cache 变量'CMAKE_INCLUDE_PATH'和'CMAKE_SYSTEM_INCLUDE_PATH'指定的路径
4. 系统环境变量'CMAKE_INCLUDE_PATH', 'INCLUDE'和'PATH'指定的路径.
5. 由`PATHS`和'HINTS'选项指定的路径
6. 以上所有路径的由'PATH_SUFFIXES'指定的子目录
````
`find_library`和`find_path`用法与功能都类似, 不过是寻找的路径中由"include"换成了"lib", 变量中的'INCLUDE'部分换成"LIBRARY". 详细的说明可以参考官方文档[find_path文档](https://cmake.org/cmake/help/v3.5/command/find_path.html)和[find_liarary文档](https://cmake.org/cmake/help/v3.5/command/find_library.html).

##### c) 使用`find_package_handle_standard_args()`
`find_package_handle_standard_args`的用法如下:
````
find_package_handle_standard_args(<PackageName>
  [FOUND_VAR <result-var>]
  [REQUIRED_VARS <required-var>...]
  [VERSION_VAR <version-var>]
  )
````
该函数将会一一检查输入的`REQUIRED_VAR`中的变量是否都有效(不以`NOTFOUND`结尾), 如果有效, 将会把`<result_var>`设置为true; `<version_var>`填实际找到的版本变量, 函数也会将这个值和`find_package`中要求的值进行比较.

#### 2. `Find<package>.cmake`例子
这里提供一个使用Module模式找glog的例子:
````
# 先使用pkg-config找包
find_package(PkgConfig)
pkg_check_modules(PC_GLOG QUIET libglog)

if (${PC_GLOG_FOUND})
# 如果pkg-config找到的话, 给find_packge需要返回的变量赋值
  set(Glog_FOUND ${PC_GLOG_FOUND})
  set(Glog_VERSION ${PC_GLOG_VERSION})
  set(Glog_INCLUDE_DIRS ${PC_GLOG_INCLUDE_DIRS})
  set(Glog_LIBRARIES ${PC_GLOG_LIBRARIES})
else ()
# 没有找到的话, 使用find_path 和find_library找
  find_path(Glog_INCLUDE_DIR
    NAMES glog
    PATHS ${PC_GLOG_INCLUDE_DIRS})
  find_library(Glog_LIBRARY_DIR
    NAMES glog
    PATHS ${PC_GLOG_LIBRARY_DIRS})

  include(FindPackageHandleStandardArgs)
# 检查是否找到, 并给<package>_FOUND变量赋值
  find_package_handle_standard_args(Glog
    FOUND_VAR Glog_FOUND
    REQUIRED_VARS Glog_INCLUDE_DIR Glog_LIBRARY_DIR
    VERSION_VAR PC_GLOG_VERSION)

  if (${Glog_FOUND})
    set(Glog_INCLUDE_DIRS ${Glog_INCLUDE_DIR})
    set(Glog_LIBRARIES ${Glog_LIBRARY_DIR})
    set(Glog_VERSION ${PC_GLOG_VERSION})
  endif ()
  mark_as_advanced(Glog_LIBRARY_DIR Glog_INCLUDE_DIR)
endif ()

if (${Glog_FOUND})
  message(STATUS "Found Glog include = ${Glog_INCLUDE_DIRS}")
  message(STATUS "Found Glog lib = ${Glog_LIBRARIES}")
  message(STATUS "Found Glog version: " ${Glog_VERSION})
else ()
  message(SEND_ERROR "Could not find Glog")
endif ()
````
完整的找包的例子可以参考[find_package_demo](./find_package_demo/)
## 二. 找包的一些小技巧
* `find_package(<package_name>)`中填的包名`<package_name>`很重要, 实际就是去找`Find<package_name>.cmake`文件或者`<package_name>Config.cmake`文件, 这是区分大小写的. 如果不知道包名填什么, 就去`/usr`下搜索一下库名, 如果能发现`xxxConfig.cmake`, 那么xxx就是你应该填的名子.
* 实际上大多数包都是使用Config模式找到的. 如果电脑上一个包存在多个版本, 一定要填需要的版本号.
* 如果找不到想要的库, 先确定电脑是否安装了这个库. 库的位置一般在`/usr`下和`/opt`下, 去这两个目录下搜索一下就能知道. 如果电脑确实安装了该库, 那么就需要根据上面讲的原理一步一步排查. 首先确定是使用Config模式还是Module模式, 然后根据模式去检查上面提到的搜索路径, 是否包含库实际存在的路径. 一般可以通过设置cmake变量`CMAKE_PREFIX_PATH`来包含实际库的路径.
