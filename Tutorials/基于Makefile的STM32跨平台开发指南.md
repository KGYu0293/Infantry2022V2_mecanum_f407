# 基于Makefile的STM32跨平台开发指南

## 软件安装及环境配置

系统要求：Windows Mac Linux均可

已经尝试过并成功的系统：
- Windows 10 20H2等多个win10版本
- 部分win11版本
- Ubuntu 20.04

### STM32Cube运行环境：Java
请尽量安装Java最新版。不要安装Java 8
#### Windows
前往[下载最新版Java](https://www.oracle.com/java/technologies/downloads/)（这里是Java17）。具体在Java SE Development Kit 17.x.x downloads项目下找到Windows，然后下载x64 Installer，一路下一步安装即可。
#### Linux 
  - 方法1：打开终端，输入`sudo apt-get insall openjdk-17-jdk`安装
  - 方法2：去上方官网下载

### STM32CubeMX：
#### 软件安装
前往ST官网https://www.st.com/en/development-tools/stm32cubemx.html下载，有Linux、Mac和Windows版本。注意需要注册并登陆一个ST账号。请务必下载最新版（修复了CAN的BUG）。

（Linux安装好之后可以自己创建一个快捷方式。建议安装在home目录下。）

下载完成后加载CMSIS库，[官网下载地址](https://github.com/ARM-software/CMSIS_5/releases/tag/5.8.0)

下载pack后缀的版本，放在容易找到的位置

#### 必要库
打开stm32cubemx软件，点击HELP,在弹出菜单中点击Manage embedded software packages，弹出的窗口中点击左下角From local ...，找到之前下载的.pack后缀文件并打开，等待其自动加载即可

![img](jpgs\cmsis_pack.png)

同时在安装Manage embedded software packages窗口中安装其他一些必要库，例如STM32Cube MCU Packages中需要使用的芯片系列的包（每个系列安装最新的即可）

### 编译器：gcc-arm-none-eabi

#### Windows：
前往[下载地址](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) 下载最新版并安装。请选择后缀win32的exe安装包。

安装过程中，在结束页面请务必勾选“Add path to environment variable”复选框。（建议全部勾选）。

![img](jpgs\image-20211231161023914.png)

完成后打开cmd或者powershell，输入`arm-none-eabi-gcc -v`

如果最后一行显示了gcc version xx.x.x 这样的就表示成功添加了环境变量。

如果提示找不到则需要手动添加，如果找到了，可直接跳转至下一环节

手动添加步骤如下：

打开设置-系统-关于，找到高级系统设置：

![img](jpgs\wps26BF.tmp.jpg) 

点击“环境变量”：

![img](jpgs\wps26C0.tmp.jpg) 

找到path，点击“编辑”：

![img](jpgs\wps26C1.tmp.jpg) 

双击击最下面的空白行，显示文本框后，点击“浏览”：

![img](jpgs\wps26C2.tmp.jpg) 

找到你的安装目录（一般是C:\Program Files (x86)\GNU Arm Embedded Toolchain\(版本号)\），选中bin文件夹，并点确定。

重复上述，再添加（安装目录）\arm-none-eabi\bin文件夹。

![img](jpgs\wps26C3.tmp.jpg) 

然后全部点确定关闭窗口。

完成后再次打开cmd或者powershell，输入`arm-none-eabi-gcc -v`检查是否添加成功。

#### Linux
打开终端，输入`sudo apt-get install gcc-arm-none-eabi`执行即可。可以使用`arm-none-eabi-gcc -v`指令检查是否安装成功。（方法参考Windows）


### Make工具（仅限Windows）：gnu-mcu-eclipse-build-tools
#### windows
前往GitHub下载：https://github.com/xpack-dev-tools/windows-build-tools-xpack/releases 

找到最新版（不是Pre-Release，请务必选择Latest），点击标题进入下载页面，下载x64.zip压缩包，之后解压到C盘根目录下。

请参照上述手动添加环境变量的方法将你解压的文件夹中的bin目录添加到环境变量（一般路径是C:\xpack-windows-build-tools-(版本号)\bin）。

完成后再次打开cmd或者powershell，输入`make -v`检查是否添加成功。
#### Linux
Linux自带GNU Make工具，不需要这一步骤。



### 代码编辑器：VScode

前往官网https://code.visualstudio.com/下载。网页会自动根据你的操作系统确定版本。

安装好之后需要安装以下必要插件（拓展，Extensions）:
- C/C++
- C/C++ Extension Pack
- ARM
- Makefile Tools

补充说明：vscode是一款功能强大的软件，有兴趣可以自行探索。

### 调试/下载工具：Ozone

前往官网https://www.segger.com/downloads/jlink/，找到Ozone - The J-Link Debugger，选择对应操作系统下载安装即可。


## 工程配置：

### 使用STM32Cube新建一个工程
按照通常的方法配置。

- 在**Project Manager**选项卡中，IDE选择Makefile：
![img](jpgs\wps7513.tmp.jpg) 

- **Code generator**这样勾选：
![img](jpgs\image-233.png)

然后生成工程。

使用VScode打开工程文件夹（整个文件夹）。在打开之前请先确认你已经安装了必要插件，并配置好了环境变量。部分版本的VScode需要你选择信任此文件夹。如果没有问题的话，VScode的输出界面应该会出现以下提示：

![img](jpgs\wps7514.tmp.jpg) 

出现这个提示代表工程已经完成了。

在左侧的侧边栏中找到Makefile选项，可以找到build按钮，点击即可编译。

![img](jpgs\wps7515.tmp.jpg) 

编译完成后会显示以下输出：

![img](jpgs\wps7516.tmp.jpg) 

此时在工程目录下即可找到build文件夹，文件夹内有后缀elf的调试用可执行文件和后缀bin的二进制烧录文件。

### 添加新源代码和头文件目录：

使用VScode打开工程目录下的Makefile文件。

1. 添加C源文件

![img](jpgs\6.png)

在这里参照上面的格式可以添加源代码文件。请注意，不要直接添加到C_SOURCES = \ ...这个由stm32cubemx生成的代码中，而是在它的下方使用C_SOURCES += 来添加你的文件。如下所示：
![img](jpgs\5.png)

2. 添加头文件目录

![img](jpgs\wps7518.tmp.jpg) 

在这里参照上面的格式可以添加头文件目录。请注意，不要直接添加到C_INCLUDES= \ ...这个由stm32cubemx生·成的代码中，而是在它的下方使用C_INCLUDES += 来添加你的文件。请在目录前加上-I前缀。

在添加到指定用户添加的位置时，使用STM32Cube重新生成工程并不会影响用户已经手动添加的文件和目录。

### 拓展：C++/C混合编译
 
修改Makefile使c++也能编译：

1. 添加一个变量存储c++源文件位置：
在ASM sources上方即可。
以后新c++的文件在此处定义，新的头文件目录直接在上一章节的C_INCLUDES+=中定义
![img](jpgs\1.png)
2. 添加c++的编译器：
在binaries区域中模仿添加g++编译器，变量名为CCP
![img](jpgs\2.png)
3. 添加c++链接库链接：
在LDFLAGS中添加即可。
![img](jpgs\3.png)
4. 添加g++编译命令：
如图，模仿gcc编译添加g++编译和链接命令。
至此，c++/c以可以混合编译了。
![img](jpgs\4.png)

关于c/c++互相调用的内容，参考[以下文章](https://segmentfault.com/a/1190000012221570)


附上改好的一份makefile文件[示例](https://github.com/cxnaive/MakeFile-for-CubeMX)

可根据其理解上述内容。

## 编译 下载与调试
### 编译
安装完成makefile tool插件后可以直接在vscode左侧点击makefile-编译按钮(上面已有示例)。也可以点击vscode最上面一行终端-运行任务-启动下载调试器。该功能并不一定能在所有工程使用，可能需要在.vscode/task.json内手动配置。框架内默认已进行配置。

若改变cube设置后再次生成或使用其他电脑上生成或修改过的makefile，此时直接编译可能出现编译不通过，此时可以使用rm build命令(即删除bulid文件夹，可以手动打开工程文件夹直接删除)后再次编译。git等方式同步时若其他协作者修改了makeflie或重新生成了工程一般也需要删除build。一般三次编译之内会不报错。

### 下载 调试
方法一：使用ozone软件进行下载与调试。注意ozone只支持jlink下载器。
ozone可以和keil一样将全局变量添加到watch并实时监测，注意watch的靠右边有一行fresh，**默认是不刷新，需要自行下拉更改成需要的频率刷新**。

方法二：使用openocd进行下载与调试。可以支持多种下载器，可以在vscode内完成下载和调试，但配置复杂，且调试功能不如ozone强大。

注：该方法尚未完全解决。

配置：
1. 下载vscode插件：Cortex-Debug 
2. 下载openocd，添加环境变量
   - [openocd下载地址](https://github.com/openocd-org/openocd/releases/tag/v0.11.0)
   - 下载后解压，将所得文件夹放到合适的位置。随后添加环境变量，与上述类似，仍是添加解压所得文件夹的bin子文件夹
3. 下载功能 .vscode/tasks.json
4. 调试功能 .vscode/launch.json

[jlink驱动下载](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack)

使用：