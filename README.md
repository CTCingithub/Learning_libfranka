# Learning `libfranka`

A record of studying libfranka, as well as other commonly-used tools.

## 1. Dependency

The dependency are shown as follows:

|Library|Version|
|:---:|:---:|
|[libfranka](https://github.com/frankaemika/libfranka)|$0.15.0$|
|[yaml-cpp](https://github.com/jbeder/yaml-cpp)|$0.8.0$|
|[Eigen](https://eigen.tuxfamily.org/dox/)|$\geq 3.3.7$|

## 2. Editor Settings

I personally use *Visual Studio Code* as my editor. I swithed to [Clangd](https://clangd.llvm.org/) for code completion and other features. In order to generate workspace configuration `.clangd` file, I wrote a python script `Clangd_Setup.py`. Run the following command in terminal to collect all absolute paths:

```bash
python Clangd_Setup.py -std=c++17 -isystem/usr/include/eigen3
```
