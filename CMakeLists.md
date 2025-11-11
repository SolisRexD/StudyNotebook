# 这份笔记总结cmake如何使用
主要模块结构如下
```
# 项目需求
cmake_minimum_required(VERSION 3.10)
project(MyProject)

# 编译选项
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -O2 -fPIC)

# 头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include)
# 源文件定义
aux_source_directory(src SRC_FILES)

# 生成可执行文件或动态库
add_executable(main ${SRC_FILES})
# 或
add_library(detector SHARED ${SRC_FILES})

# 链接依赖库
target_link_libraries(main ${OpenCV_LIBS} pthread MxBase)
```
## 项目需求
```
# 项目需求
cmake_minimum_required(VERSION 3.10)
project(MyProject)
```
定义最低需求版本
项目名,项目语言
project的完整写法如下
```
project(
    MyProject                # 工程名
    VERSION 1.2.3            # 工程版本
    LANGUAGES C CXX          # 使用哪些语言
    DESCRIPTION "UAV detect on Atlas"
    HOMEPAGE_URL "https://example.com"
)
```
若有子目录,子项目
```
add_subdirectory(third_party/lib1)
add_subdirectory(src)
```
此时子项目下也会有个project

## 编译选项
可以设置编译标准,例如
```
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) #强制要求编译器支持,若不支持,直接报错
set(CMAKE_CXX_EXTENSIONS OFF)  # 可选：禁用编译器私有扩展，强制用标准 c++
```
全局编译选项
```
add_compile_options(-Wall -O2 -fPIC)
```
对应g++的
```
g++ -Wall -O2 -fPIC ...
```
让GPT协助总结参数,供参考
| 分类               | 参数                               | 功能 / 含义        | 人话解释                      | 建议使用场景            |
| ---------------- | -------------------------------- | -------------- | ------------------------- | ----------------- |
| ⚠️ **警告与错误控制**   | `-Wall`                          | 开启大多数常见警告      | “帮我挑出常见问题”                | ✅ 永远推荐            |
|                  | `-Wextra`                        | 开启额外警告         | “再严格点，多报些可疑点”             | ✅ 建议与 `-Wall` 一起  |
|                  | `-Werror`                        | 将警告视为错误        | “有问题就不让编译通过”              | ⚠️ 团队规范或成熟项目      |
|                  | `-Wno-unused-variable`           | 关闭特定警告         | “变量没用不报错”                 | 调试阶段偶尔用           |
|                  | `-Wshadow`                       | 警告变量名遮蔽        | “局部变量和成员变量同名”             | 规范性较高时使用          |
| 🧩 **优化控制**      | `-O0`                            | 不优化            | “调试方便但慢”                  | Debug 模式          |
|                  | `-O1`                            | 轻度优化           | “小优化”                     | 很少单独用             |
|                  | `-O2`                            | 平衡优化（常用）       | “性能好又稳定”                  | ✅ Release 默认      |
|                  | `-O3`                            | 激进优化           | “尽可能快，但不一定稳”              | 性能测试              |
|                  | `-Ofast`                         | 放弃部分标准保证       | “牺牲兼容性换速度”                | 科学计算 / 浮点性能场景     |
|                  | `-Os`                            | 优化体积           | “嵌入式代码更小”                 | 资源受限设备            |
|                  | `-Og`                            | 调试优化           | “调试 + 一点点优化”              | ✅ Debug 推荐        |
|                  | `-march=native`                  | 针对本机 CPU 优化    | “用上当前 CPU 的全部指令集”         | ⚠️ 仅本机测试时用        |
| 🧠 **调试与断言**     | `-g`                             | 生成调试信息         | “让 gdb 看得懂源码”             | ✅ Debug 模式必加      |
|                  | `-DNDEBUG`                       | 禁用断言           | “Release 模式关掉 assert”     | ✅ Release 模式      |
|                  | `-DDEBUG`                        | 开启调试宏          | “让代码里 `#ifdef DEBUG` 生效”  | 调试阶段              |
| 🔐 **安全加固**      | `-fstack-protector`              | 栈保护（轻量）        | “函数栈上放金丝雀”                | ✅ 推荐              |
|                  | `-fstack-protector-all`          | 栈保护（所有函数）      | “每个函数都加保护”                | ✅ 安全项目            |
|                  | `-D_FORTIFY_SOURCE=2`            | glibc 检查函数边界   | “检查 strcpy、sprintf 越界”    | ✅ 与 `-O2` 配合      |
|                  | `-fPIE`                          | 生成位置无关可执行代码    | “支持 ASLR 的可执行文件”          | ✅ 安全加固（可执行）       |
|                  | `-fPIC`                          | 生成位置无关代码       | “动态库必需”                   | ✅ 做 .so 必加        |
| ⚙️ **代码生成与 ABI** | `-fno-common`                    | 防止全局符号重复定义     | “不允许重复全局变量”               | 规范项目              |
|                  | `-fvisibility=hidden`            | 默认隐藏符号         | “减少导出符号，保护接口”             | 库开发常用             |
|                  | `-D_GLIBCXX_USE_CXX11_ABI=0`     | 使用旧版 C++ ABI   | “兼容老库（如 MindX SDK）”       | ✅ Atlas 平台必加      |
| 🧩 **C++ 特性裁剪**  | `-fno-exceptions`                | 禁用异常机制         | “不允许 throw/catch”         | ⚠️ 嵌入式场景          |
|                  | `-fno-rtti`                      | 禁用运行时类型信息      | “不能用 dynamic_cast/typeid” | ⚠️ 轻量化优化          |
|                  | `-std=c++11` / `c++14` / `c++17` | 指定语言标准         | “选定 C++ 版本”               | ✅ 必填              |
|                  | `-fno-omit-frame-pointer`        | 保留栈帧指针         | “方便性能分析/回溯”               | Debug / Profiling |
| 🧰 **工程诊断与静态检查** | `-ftime-report`                  | 输出编译耗时统计       | “看哪个阶段耗时多”                | 性能调优编译器用          |
|                  | `-save-temps`                    | 保留中间文件（.i, .s） | “方便分析编译中间产物”              | 学习/调试编译过程         |
|                  | `-fsyntax-only`                  | 仅检查语法不生成代码     | “快速检测语法错误”                | CI / Lint 阶段      |
|                  | `-E`                             | 只运行预处理         | “展开宏，查看宏替换结果”             | 调试宏定义             |

另外,可以为每个目标设置单独的编译选项
```
target_compile_options(my_target PRIVATE -Wall -O2 -fPIC)
```
## 头文件和源文件
定义头文件目录
```
include_directories(${PROJECT_SOURCE_DIR}/include)
```
等同
```
g++ -I /path/to/project/include ...
```
常见用法
| 写法                                                                 | 含义                  | 举例                                     |
| ------------------------------------------------------------------ | ------------------- | -------------------------------------- |
| `include_directories(include)`                                     | 相对当前 CMakeLists 的路径 | `src/CMakeLists.txt` 中找 `src/include/` |
| `include_directories(${PROJECT_SOURCE_DIR}/include)`               | 工程根目录的 include 文件夹  | 最常见                                    |
| `include_directories(${CMAKE_SOURCE_DIR}/third_party/foo/include)` | 引入第三方库的头文件目录        | 外部依赖                                   |
| `include_directories(SYSTEM /usr/local/include)`                   | 声明为系统路径（警告更少）       | 第三方头文件                                 |

扫描源文件
扫描指定目录（src/）下所有 .c 或 .cpp 文件，
并把它们的路径存进变量 SRC_FILES。
```
aux_source_directory(src SRC_FILES)
```