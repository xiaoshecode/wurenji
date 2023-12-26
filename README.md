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
step1:  `chmod u+x *.sh`

setp2:  `./startpsdk.sh`

step3:  `./startroscore.sh`

step4:  `./startuavservice.sh`

step5:  `./startimagepub.sh`

step6:  `./startpass_ring.sh`