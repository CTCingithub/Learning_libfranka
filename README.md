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

I personally use *Visual Studio Code* as my editor. In order to avoid unnecessary editor warnings, the `c_cpp_properties.json` is defined as:

```json
{
  "configurations": [
    {
      "name": "linux-gcc-x64",
      "includePath": [
        "${workspaceFolder}/**",
        "${workspaceFolder}/**/include/**",
        "/usr/include/**",
        "/usr/include/**/**",
        "/usr/local/include/**"
      ],
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "${default}",
      "cppStandard": "${default}",
      "intelliSenseMode": "linux-gcc-x64",
      "compilerArgs": [
        "-std=c++17"
      ]
    }
  ],
  "version": 4
}
```
