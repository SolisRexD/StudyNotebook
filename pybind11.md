# 环境准备
```
# 安装
pip install pybind11

# 查看包含目录（确认路径）
python -m pybind11 --includes
```

# 项目结构
```
demo/
├─ CMakeLists.txt
└─ add.cpp
```
# 示范函数
使用PYBIND11_MODULE,使得Python可以调用
```
#include <pybind11/pybind11.h>
namespace py = pybind11;

int add(int a, int b) { return a + b; }

PYBIND11_MODULE(myadd, m) {          // 生成 Python 模块名：myadd
    m.doc() = "A tiny add module";   // 模块文档（可选）
    m.def("add", &add, "Add two integers",
          py::arg("a"), py::arg("b"));  // 函数绑定
}
```
# 编译
cmakelists示例
```
cmake_minimum_required(VERSION 3.14)
project(myadd LANGUAGES CXX)

# 找 pybind11（方式一：系统/虚拟环境内安装的 CMake config）
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(myadd add.cpp)   # 生成 Python 扩展模块
set_property(TARGET myadd PROPERTY CXX_STANDARD 14)
```
编译
```
cd demo
cmake -B build -S .
cmake --build build -j

# 生成的 myadd.*.so 在 build/ 目录中（不同平台后缀略有差异）
python -c "import myadd; print(myadd.add(3,4))"   # => 7
```
# 类
```
#include <pybind11/pybind11.h>
namespace py = pybind11;

class Projector {
public:
    explicit Projector(double fx, double fy, double cx, double cy)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

    // 演示：像素→相机坐标（简单示例）
    std::pair<double,double> pixel_to_norm(double u, double v) const {
        double x = (u - cx_) / fx_;
        double y = (v - cy_) / fy_;
        return {x, y};
    }

private:
    double fx_, fy_, cx_, cy_;
};

PYBIND11_MODULE(geom, m) {
    py::class_<Projector>(m, "Projector")
        .def(py::init<double,double,double,double>(),
             py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"))
        .def("pixel_to_norm", &Projector::pixel_to_norm,
             py::arg("u"), py::arg("v"),
             "Convert pixel (u,v) to normalized camera coords (x,y)");
}
```
# 使用pybind11的数据类型,可以最高效传输
```
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
namespace py = pybind11;

// 输入: HxW uint8；输出: HxW uint8
py::array_t<uint8_t> invert(py::array_t<uint8_t> img) {
    // 请求 buffer（检查维度）
    py::buffer_info buf = img.request();
    if (buf.ndim != 2) throw std::runtime_error("Expect 2D grayscale image");

    auto H = buf.shape[0], W = buf.shape[1];
    auto out = py::array_t<uint8_t>({H, W});
    py::buffer_info outbuf = out.request();

    uint8_t* inptr  = static_cast<uint8_t*>(buf.ptr);
    uint8_t* outptr = static_cast<uint8_t*>(outbuf.ptr);

    // 简单逐像素操作
    for (ssize_t i=0; i<H*W; ++i) outptr[i] = 255 - inptr[i];
    return out; // 返回给 Python 端仍是 ndarray
}

PYBIND11_MODULE(imgops, m) {
    m.def("invert", &invert, "Invert grayscale image");
}
```
# 函数定义的几种方式
| 形式                                                             | 用途               |
| -------------------------------------------------------------- | ---------------- |
| `m.def("f", &func);`                                           | 最基本绑定            |
| `m.def("f", &func, "doc");`                                    | 加文档字符串           |
| `m.def("f", &func, py::arg("x"), py::arg("y")=1);`             | 命名 + 默认值         |
| `m.def("f", [](int x){ return x*2; });`                        | 直接绑定 lambda      |
| `m.def("f", &func, py::call_guard<py::gil_scoped_release>());` | 调用时释放 GIL（多线程友好） |
| `m.def("f", &func, py::return_value_policy::reference);`       | 控制返回值所有权（比如返回指针） |

还可以使用py::function py_fun这样的方式作为参数，这样可以进行回调

# 类定义的几种方式
链式定义
```
# 假设我们有个Point类
PYBIND11_MODULE(geom, m) {
    py::class_<Point>(m, "Point")           // ← 把 C++ 类 Point 暴露为 Python 类 geom.Point
        .def(py::init<double,double>())     // 构造函数 __init__(x, y)
        .def("distance", &Point::distance)  // 成员函数绑定
        .def("to_string", &Point::to_string)// 成员函数绑定
        .def_readwrite("x", &Point::x_)     // 暴露公有成员变量
        .def_readwrite("y", &Point::y_);
}
```
基本结构
```
py::class_<C++类>(模块对象, "Python名字", "可选docstring")
```
| 写法                                    | 意义                  | 对应 Python          |
| ------------------------------------- | ------------------- | ------------------ |
| `.def(py::init<>())`                  | 绑定默认构造函数            | `obj = Class()`    |
| `.def(py::init<int,int>())`           | 绑定带参数构造函数           | `obj = Class(1,2)` |
| `.def("func", &Class::func)`          | 绑定成员函数              | `obj.func()`       |
| `.def_static("f", &Class::f)`         | 绑定静态函数              | `Class.f()`        |
| `.def_readwrite("x", &Class::x_)`     | 可读可写成员变量            | `obj.x`            |
| `.def_readonly("id", &Class::id_)`    | 只读成员变量              | 只能读不能写             |
| `.def_property("x", getter, setter)`  | 自定义属性访问             | 支持 get/set 拦截      |
| `.def("__repr__", &Class::to_string)` | 自定义 Python `repr()` | print 对象时的显示       |
