## 如何安装cmake Config Package
camke安装主要是使用`install()`指令, 然后使用`CMakePackageConfigHelpers`Module生成相应的配置文件和版本文件.
### 1. install()指令
`install()`指令可以安装Targets, 也可以安装文件.详细的指令可以参考[install 说明](https://cmake.org/cmake/help/v3.5/command/install.html#installing-exports)
install Targets的简单用法为:
````
install(TARGETS targets...
        [[ARCHIVE|LIBRARY|RUNTIME|
         [DESTINATION <dir>]
         [CONFIGURATIONS [Debug|Release|...]]
        ] [...]
        )
````
其中 `TARGETS`选项后面填一系列的target名字(可执行文件或库文件); `ARCHIVE`对应着静态链接库类型, `LIBRARY`对应着动态链接库类型, `RUNTIME`对应着可执行类型; `DESTINATION`后面填要安装的目录, 相对于变量`CMAKE_INSTALL_PREFIX`指定的路径(**默认为`/usr/local`**), 也就是说相应的target会被安装到`${CMAKE_INSTALL_PREFIX}/<dir>`目录下.

install Files 的简单用法为:
````
install(FILES files...
         [DESTINATION <dir>]
        )
````
### 2. 生成配置文件
借助于cmake提供的`CMakePackageConfigHelpers`Module可以帮助我们生成版本文件和配置文件. 详细说明可以参考[CMakePackageConfigHelpers官方说明文档](https://cmake.org/cmake/help/v3.5/module/CMakePackageConfigHelpers.html).
#### a) 生成版本文件
````
write_basic_package_version_file(<filename>
  [VERSION <major.minor.patch>]
  COMPATIBILITY <AnyNewerVersion|SameMajorVersion|ExactVersion> )
````
* `<filename>`填要生成的版本文件的名字, 要写成`<package>ConfigVersion.cmake`的形式. 该项可以填绝对路径, 否则认为是相对于`CMAKE_CURRENT_BINARY_DIR`的相对路径. 最终就在指定目录下生成该版本文件.
* `VERSION` 后面填版本号
* `COMPATIBILITY`后面跟的选项决定了找包时版本匹配的规则. `AnyNewerVersion`表示只要`find_package`要求的版本比实际版本低就算匹配上; `SameMajorVersion`表示需要`find_package`要求的版本中第一个数字与实际版本的第一个数字一样就算匹配上; `ExactVersion`表示要`find_package`要求的版本和实际版本数字完全一样才算匹配上.

#### b) 生成配置文件
````
configure_package_config_file(<input> <output>
  INSTALL_DESTINATION <path>
  [PATH_VARS <var1> <var2> ... <varN>]
  [INSTALL_PREFIX <path>]
  )
````
1. 需要自己写一个`<package>Config.cmake.in`文件作为`<input>`. 文件中一开始可以设置版本变量, 然后需要写一行`@PACKAGE_INIT@`, 该函数会将这个字符串替换为两个`marcos`的定义: `set_and_check()`和`check_required_conmpnents()`, 之后就可以在文件中使用这两个`marco`. 前者具有`set()`的功能, 并且会检查是否有效.我们需要在`<package>Config.cmake.in`文件中设置`find_package`需要的标准变量`<package>_INCLUDE_DIRS, <package>_LIBRARIES`
2. `<output>`填生成的文件, 要写为`<package>Config.cmake`的形式. 该项可以填绝对路径, 否则认为是相对于`CMAKE_CURRENT_BINARY_DIR`的相对路径. 最终就在指定目录下生成该版本文件.
3. `INSTALL_DESTINATION` 要填最终`<package>Config.cmake`会被安装的位置. 可以是绝对路径, 也可以是相对于`<INSTALL_PREFIX>`项指定的路径. `INSTALL_PREFIX`不填的话默认为`CMAKE_INSTALL_PREFIX`. 之后也必须用`install()`将配置文件安装到`INSTALL_DESTINATION`指定的位置.
4. `PATH_VARS`后面填需要被替换的变量的名字. 比如填了`PATH_VARS INCLUDE_DIRS`, 那么该函数就会将`<input>`file中的`@PACKAGE_INCLUDE_DIRS@`替换为`INCLUDE_DIRS`.

### 3. 例子
下面给出一个安装tinysplin的例子.
首先写一个`tinysplineConfig.cmake.in`文件:
````
set(tinyspline_VERSION 0.0.1)
SET(tinyspline_VERSION_MAJOR  0)
SET(tinyspline_VERSION_MINOR  0)
SET(tinyspline_VERSION_PATCH  1)
SET(tinyspline_VERSION_TWEAK  0)

@PACKAGE_INIT@
set_and_check(tinyspline_INCLUDE_DIRS "@PACKAGE_INCLUDE_INSTALL_DIR@")
set_and_check(tinyspline_LIBRARIES "@PACKAGE_LIB_INSTALL_DIR@")
set_and_check(tinyspline_CONFIG "@PACKAGE_CONFIG_INSTALL_DIR@")
````
然后写`CMakeLists.txt`文件:
````
cmake_minimum_required(VERSION 3.3)
project(install_package_demo)

set(CMAKE_CXX_STANDARD 11)

include_directories(
  include
)

aux_source_directory(src LIB_SOURCE_FILES)
file(GLOB_RECURSE LIB_HEADER_FILES include/*.h)

add_library(tinyspline
  ${LIB_SOURCE_FILES})

add_executable(${PROJECT_NAME}
  main.cpp)
target_link_libraries(${PROJECT_NAME}
  tinyspline)

# change install prefix
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install")
message("CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})
# install targets
install(TARGETS tinyspline ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
# install header files
install(FILES ${LIB_HEADER_FILES}
  DESTINATION include)

set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(CONFIG_INSTALL_DIR "${LIB_INSTALL_DIR}/cmake")

# generate config file and version file using CMakePackageConfigHelpers
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  tinysplineConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  cmake/tinysplineConfig.cmake.in
  tinysplineConfig.cmake
  INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
  PATH_VARS LIB_INSTALL_DIR INCLUDE_INSTALL_DIR CONFIG_INSTALL_DIR)

#install .cmake files
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tinysplineConfigVersion.cmake ${CMAKE_CURRENT_BINARY_DIR}/tinysplineConfig.cmake
  DESTINATION ${CONFIG_INSTALL_DIR})
````
完整的例子在[install_package_demo](./install_package_demo)中, 可以使用cmake编译安装后观察生成的文件内容和安装位置.
