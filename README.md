<div align="center">


始于2023.11.19 - 止于2024.8.17

---

# 百度完全模型

上交AuTop战队开源方案移植

_✨ 推荐使用CMake 3.12 ✨_

<a href="https://raw.githubusercontent.com/luoy-oss/BaiduEdgeboard/main/LICENSE">
    <img src="https://img.shields.io/github/license/luoy-oss/BaiduEdgeboard" alt="license">
</a>
<img src="https://img.shields.io/badge/c++ 17-blue?logo=C&logoColor=edb641" alt="C">
<img src="https://img.shields.io/badge/4.14.0_xilinx-v2018.3-blue?logo=linux&logoColor=edb641" alt="C">

<br />

</div>

## 运行主程序

> 克隆本仓库
```
git clone https://github.com/luoy-oss/BaiduEdgeboard.git
```

<br>

> 进入主目录
```
cd BaiduEdgeboard
```

<br>

> 创建build文件夹并进入
```
mkdir build
cd build
```

<br>

> cmake编译
```
cmake ../src
make baidu -j10
```

<br>

> 运行主程序
```
sudo ./baidu
```

<br>

## Camera参数调试程序

> cmake编译
```
cmake ../src
make camera -j10
```

<br>

> 运行相机参数调试程序
```
sudo ./camera
```


