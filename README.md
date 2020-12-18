# 配置OSketch编译环境

## 安装开发软件

- Visual Studio，版本不要低于图中所示的15.9.11![VSVersion](doc/VSVersion.png)

- [CMake](https://cmake.org/)
- [git](https://git-scm.com/download/win)
- [VSCode](https://code.visualstudio.com/)，用于修改文件编码
- [GitKraken](https://www.gitkraken.com/)，用于可视化的管理 git 代码库
- IDE 最好选一个等宽字体，比如 [monaco字体](https://github.com/todylu/monaco.ttf)
- [Qt Vs Tools](https://marketplace.visualstudio.com/items?itemName=TheQtCompany.QtVisualStudioTools-19123),是 Qt 针对 Visual Studio 的一个插件，需要安装针对性的 VS 版本

## 下载插件代码

- 从 Github 上下载开发环境，在 OSketchProPlugin 的 Release 页面下，下载最新版本。[https://github.com/saedrna/OSketchProPlugin/releases](https://github.com/saedrna/OSketchProPlugin/releases)

- 输入下列命令，将代码下载到本机，假定路径如下 ```%OSKETCH_DIR%```

```bash
git clone https://github.com/saedrna/OSketchProPlugin.git
```

- 自己编写代码的时候，必须确保编码格式为UTF-8 with BOM，可以在vscode里面把默认encoding设为utf8bom，在创建文件时候就会自动为UTF-8 with BOM，只有cpp文件需要修改编码，cmakelist ui 等文件，就用默认的不要修改。

![FileEncoding](doc/FileEncoding.png)

## 配置第三方库环境

- 安装 [miniconda](https://docs.conda.io/en/latest/miniconda.html)或者[anaconda](https://www.anaconda.com/download/)
- 配置 anaconda，并且创建一个第三方库环境osketch:

```
conda config --add channels conda-forge
conda config --add channels saedrna

cd %OSKETCH_DIR%

conda env create -n osketch -f osketch-fix-version.yml
```

- 创建自己的开发分支，命名一个自己的分支名 ```your_branch_name```

```
git checkout -b your_branch_name
```

- 完成后，拷贝一些文件，注意拷贝文件时不要退出上文中创建的osketch环境。

```
conda activate osketch
cd toolchains
robocopy ./ %CONDA_PREFIX% h2o.cmake
robocopy ./ %CONDA_PREFIX%/Library/plugins applocal.ps1
robocopy ./ %CONDA_PREFIX%/Library/plugins qtdeploy.ps1
robocopy ./ %CONDA_PREFIX% applocal.ps1
```
- 编辑```/Library/share/openMVG/cmake/OpenMVGConfig.cmake``` 文件，这个文件有BUG，这里加一个 ../

![OpenMVGBug](doc/OpenMVGBug.png)

## 配置 CMake

- 启动 Anaconda 的控制台
- 修改 ```OSketchProPlugin/build.bat``` 文件，将 pushed 后的路径修改为本机vs安装路径，DCMAKE_INSTALL_PREFIX 路径修改为 dev 文件夹目录
![DevFolder](doc/build.png)

- 运行下列命令生成 VS 工程文件

```
cd %OSKETCH_DIR%
conda activate osketch
build.bat
```

- 启动 build 目录下的 VS 工程
```
start ./build/OSketchPlugins.sln
```

## 编译代码

- 需要选择 Release 或 RelWithDebInfo 模式，默认的 Debug 模式不行
- 编译完成后，可能需要手动拷贝一些 dll 文件，这些文件在 dev 下的 ruby/dlls 下面，包括
  - QT 的 plugins，在 ```%CONDA_PREFIX%/Library/plugins``` 目录下，包括qt.conf文件，需要手动拷贝到运行程序根目录。如果直接复制dll下的文件提示有 qt 版本问题，请使用本机该目录下的qt.conf文件进行替换并重新生成。
  - osg 的 plugins，在 ```%CONDA_PREFIX%/Library/bin/osgPlugins-3.4.1``` 目录下
  - dev 目录 bin 下面的动态库 
  - ```depends/dog/bin``` 下面的文件
  - ruby 动态库文件 ```x64-msvcrt-ruby220.dll```，在 SketchUp 安装目录下
  - SketchUp 插件动态库，在 ```depends/SketchUpAPI/bin```目录下
  - 如果还有缺失文件，再次编译后，会自动拷贝过来

![CopyDLLs1](doc/CopyDLLs1.png)
![CopyDLLs2](doc/CopyDLLs2.png)

## 示例：导入插件的编译
- 运行这一步之前需有相应的加密狗

- 修改 OSketchProPlugin\src 路径下对应的CMakeLists文件，加载 osketchpro_plugin_import 插件。![src-cmakelist](doc/src-cmakelist.png)

- 修改  OSketchProPlugin\src\osketchpro_plugin_import路径下对应的 CMakeLists 文件。其中，将第一处和第三处的 ```{CMAKE_CURRENT_LIST_DIR}``` 修改为 ```{CMAKE_INSTALL_PREFIX}``` 并去掉``` /../..``` ，第二处的两行前添加 ```H2O::```
![import-cmakelist](doc/import-cmakelist.png)

- 在VS中，右键选中 osketchpro_plugin_import 项目，打开属性，修改命令下的地址为sketchup.exe ，并右键将该项目设为启动程序。

![importproperty](doc/importproperty.png)
- 最终运行结果如下图。
![importresult](doc/importresult.jpg)

## 插件的开发

- 此处以开发一个简单插件osketchpro_plugin_helloworld为例。具体功能为：在工具栏中添加一个按钮，点击弹出窗口，显示Hello World。此处简述流程，代码与注释可点击链接查看。

- 在 ```OSketchProPlugin/src``` 目录下创建一个新插件文件夹。 ```osketchpro_plugin_helloworld```

- 创建一个插件入口类，包括一个 .cpp 源文件和一个 .h 头文件。此处我们将其分别命名为： [osketchpro_plugin_helloworld.h](src/oksetchpro_plugin_helloworld/osketchpro_plugin_helloworld.h) ，   [osketchpro_plugin_helloworld.cpp](src/oksetchpro_plugin_helloworld/osketchpro_plugin_helloworld.cpp)。这两个文件作为插件的入口，实现了插件的初始化。在本插件中，该部分主要完成了翻译文件的加载、UI信息的创建以及赋予插件ID。

- 把其他插件目录下的 ```.ts``` 文件拷贝到当前目录作为翻译文件,修改文件名为OSketchProPluginHelloWorld_zh_CN 即可。

- 使用QT Designer创建一个 ui 文件，类型为```Dialog without Buttons```,插入一个```push button```部件。为便于后续使用，我们将```QDialog```类命名为```WidgetHelloWorld```，将```QPushButton```类命名为```btn_test```。最终得到一个如下图所示的对话框，我们将该ui文件命名为```widget_hello_world.ui```，保存在```osketchpro_plugin_helloworld```文件夹下。该UI即为点击该插件时弹出的窗口。   
![widgethelloworld](doc/helloworldwidget.png)

- 创建对应的qrc资源文件。此处可使用QT Creator进行创建，也可直接创建后缀为 .qrc 的文件然后进行编辑。此处我们创建名为 ```osketchpro_plugin_helloworld```的资源文件，然后在vs2017中进行编辑。首先使用 ```Add Prefix``` 添加一个前缀，再在该前缀下使用 ```Add Files```添加所需资源。此处我们添加images文件夹，文件夹下的 building.png 将作为该插件的图标。 
![qrc](doc/qrc.png)

- 创建 ui 代码的 cpp 文件和 h 文件。对应命名为 
[widget_hello_world.h](src/oksetchpro_plugin_helloworld/widget_hello_world.h) ，
[widget_hello_world.cpp](src/oksetchpro_plugin_helloworld/widget_hello_world.cpp)。这两个文件用于具体实现各个按钮的功能，进行connect的连接。本插件中该部分通过connect连接，实现了点击 btn_test 按钮弹出写有“Hello World！”的窗口的功能。

- 创建一个 [CMakeLists.txt](src/oksetchpro_plugin_helloworld/CMakeLists.txt) 。该文件可借鉴其他部分如import插件的CMakeLists文件，将对应的名称进行修改即可。

- 上述操作完成后，仿照前一部分的插件编译教程，对osketchpro_plugin_helloworld项目进行编译。

- 双击项目中的 .ts文件打开QT Linguist界面，将缺少翻译的内容进行添加并保存。
![QTLinguist](doc/QTLinguist.png)

- 编译后进行调试，可能会出现下图中的错误：
![helloworlderror](doc/helloworlderror.png)
这是因为翻译文件中存在没有翻译的部分导致的。此时需要同上一步一样对有问题的 ts 文件进行修改。
- 最终生成结果如下图所示，工具栏中的第四项即为HelloWorld插件。新生成的插件暂时无法指定位置，会随机添加在工具栏的某一处。
![helloworldresult](doc/helloworldresult.png)