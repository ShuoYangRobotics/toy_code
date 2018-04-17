UAV\_ROS\_pkgs
================

把我们所有UAV上的package都放在这里面，这样只要在UAV上面git init就能方便管理代码了

####packages的介绍请看[wiki page](https://github.com/HKUSTUAV/UAV_ROS_pkgs/wiki)

### 第一次pull的方法
首先创建一个新的workspace
```sh
  $ mkdir -p ~/UAV_ROS_pkgs/src
  $ cd ~/UAV_ROS_pkgs/src
  $ catkin_init_workspace
  $ cd ~/UAV_ROS_pkgs
  $ catkin_make
```
为了方便将来使用，记得把这句话加到`.bashrc`里
```sh
  source ~/UAV_ROS_pkgs/devel/setup.bash
```

然后在这个workspace的src文件夹里设置git
```sh
  $ git init
  $ git remote add origin https://github.com/HKUSTUAV/UAV_ROS_pkgs.git
  $ git pull origin master
```


### Coding Style
1. package 名使用snake\_case
2. topic 名也使用snake\_case
3. 函数名、变量名使用camelCase
4. namespace，class名使用CamelCase
5. 其他使用[Google C++ coding style](http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml)
