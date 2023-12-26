# 运行顺序
作者：We are 昌平队


## 文件树
```
├─bulid
│  └─CMakeFiles
│      ├─3.10.2
│      │  ├─CompilerIdC
│      │  └─CompilerIdCXX
│      └─decodertest.dir
├─Decodertest
│  ├─include
│  ├─lib
│  └─pictures
├─image_pub
│  ├─include
│  ├─lib
│  └─src
└─raspicam_node
    ├─.github
    ├─launch
    ├─msg
    ├─src
    │  ├─cv
    │  ├─proxy_src
    │  ├─test_scripts
    │  ├─ttalink
    │  │  └─common
    │  └─utils
    └─srv
```

## 运行步骤
step0: `cd image_pub`根据readme.md编译image_pub


准备环境
step1:  `chmod u+x *.sh`

setp2:  `./startpsdk.sh`

step3:  `./startroscore.sh`

step4:  `./startuavservice.sh`

step5:  `./startimagepub.sh`

飞行
step6:  `./startpass_ring.sh`

tips: step2-6每个均需要开启一个terminal，注意step6是穿环程序，无人机会起飞，注意安全！！！

一定要确认好场地并手拿着遥控器随时准备手控的时候起飞，注意安全，注意安全，注意安全。