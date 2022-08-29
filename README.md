# SwarmMap

[![Watch the video](https://i.imgur.com/TIw235B.jpeg)](https://youtu.be/CXOAqzn6szo)

🚧 Under Construction

# 1. Related Publications

Xu, J., Cao, H., Yang, Z., Shangguan, L., Zhang, J., He, X., & Liu, Y. (2022). {SwarmMap}: Scaling Up Real-time Collaborative Visual {SLAM} at the Edge. In 19th USENIX Symposium on Networked Systems Design and Implementation (NSDI 22) (pp. 977-993).

# 2. License
SwarmMap is released under a [GPLv3 license](https://github.com/MobiSense/SwarmMap/blob/master/LICENSE.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/MobiSense/SwarmMap/blob/master/Dependencies.md).

For a closed-source version of SwarmMap for commercial purposes, please contact the author(s):  
`admin (at) sense-lab (dot) org`

If you use SwarmMap in academic work, please cite:
```BibTeX
@inproceedings{xu2022swarmmap,
  title={$\{$SwarmMap$\}$: Scaling Up Real-time Collaborative Visual $\{$SLAM$\}$ at the Edge},
  author={Xu, Jingao and Cao, Hao and Yang, Zheng and Shangguan, Longfei and Zhang, Jialin and He, Xiaowu and Liu, Yunhao},
  booktitle={19th USENIX Symposium on Networked Systems Design and Implementation (NSDI 22)},
  pages={977--993},
  year={2022}
}
```

# 3. Installation

We have tested SwarmMap with Ubuntu 18.04/20.04, CentOS 7. We recommend using a powerful computer to ensure the performance of multi-agent SLAM.

## 3.1 Install dependencies

- [spdlog](https://github.com/gabime/spdlog): log utility
- [Pangolin](https://github.com/stevenlovegrove/Pangolin): visualization (tag: v0.5)
- [Eigen3](https://gitlab.com/libeigen/eigen): optimization-related
- [CUDA](https://developer.nvidia.com/cuda-toolkit): ORB feature extraction speedup
- OpenCV with cuda (3.4.6/4.2.0 tested)
- [Boost](https://www.boost.org/): serialization (version >= 1.70.0)
- [Boost beast](https://github.com/boostorg/beast): network communication

## 3.2 Build third-party libraries

1. g2o: 
```shell
cd SwarmMap/code/Thirdparty/g2o/ && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
```

2. DBoW2
```shell
cd SwarmMap/code/Thirdparty/DBoW2/ && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
```

## 3.3 Build SwarmMap
1. Clone the repository
```shell
git clone https://github.com/MobiSense/SwarmMap
```

2. Build the project

```shell
cd SwarmMap/
mkdir build
cd build
# set your cuda path (tested version: 10.2)
cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda 
make -j
```
# 4. Usage

## 4.1 Command line arguments

### Server-client combined mode

Executable: swarm_map
```shell 
SwarmMap - Scaling Up Real-time Collaborative Visual SLAM at the Edge:
  -h, --help                print this message
  -v, --voc arg             path to vocabulary
  -d, --dataset arg         path to dataset config file
  -l, --log arg (=debug)    log level: error/warn/info/debug
  -u, --viewer arg (=1)     use frame viewer
  -m, --mapviewer arg (=1)  use map viewer
  -c, --client arg (=2)     client number
```
### Server-client standalone mode
- Client - swarm_client
  ```shell
  ./swarm_client --voc ../code/Vocabulary/ORBvoc.bin --log info --viewer 1 --mapviewer 1 --dataset PATH_TO_DATASE_FILE
  ```
- Server - swarm_server
  ```shell
  ./swarm_server --voc ../code/Vocabulary/ORBvoc.bin --log info --viewer 1 --mapviewer 1 --dataset PATH_TO_DATASE_FILE
  ```

## 4.2 Config file format
SwarmMap designs a config format to launch multi-agent SLAM, which specifies the following parameters:

mh01-03.yaml
```yaml
%YAML:1.0 
# EuRoC MH01 - 03 (3 agents) dataset config file

# dataset type [euroc/tum/kitti]
TYPE: 'euroc'
# calibration and config file path
SETTING: '/Examples/Monocular/EuRoC.yaml'
# image sequnce path (each agent's sequence path)
IMAGES: ['/MH_01_easy/mav0/cam0/data', '/MH_02_easy/mav0/cam0/data', '/MH_03_medium/mav0/cam0/data']
# timestamps file path (each agent's timestamp file)
TIMES: ['/Examples/Monocular/EuRoC_TimeStamps/MH01.txt', '/Examples/Monocular/EuRoC_TimeStamps/MH02.txt', '/Examples/Monocular/EuRoC_TimeStamps/MH03.txt']

# SERVER HOST
HOST: 'xxx.xxx.xxx.xxx'
# port the server listens (when the config file is used by the server)
# port the client connects (when the config file is used by the client)
PORT: 1234
```

kitti00-02.yaml:
```yaml
%YAML:1.0
# KITTI 00-02 (3 agents) dataset config file
TYPE: 'kitti'
SETTING: '/Examples/Monocular/KITTI00-02.yaml'
IMAGES: ['/kitti/sequences/00', '/kitti/sequences/01', '/kitti/sequences/02']
# SERVER HOST
HOST: 'xxx.xxx.xxx.xxx'
# port the server listens (when the config file is used by the server)
# port the client connects (when the config file is used by the client)
PORT: 1234
```

# 5. Examples
- Remember to set the correct path to the dataset config file and vocabulary file
- The vocabulary file SwarmMap used is in binary format with suffix `.bin` instead of the origin `.txt`
- The program binary `swarm_map` is output to the folder `bin`

## 5.1 Example on the EuRoC dataset
```shell
cd SwarmMap/bin/
./swarm_map -d ../config/mh01-03.yaml -v ../code/Vocabulary/ORBvoc.bin -c 3 -l info
```

## 5.2 Example on the KITTI dataset
```shell
cd SwarmMap/bin/
./swarm_map -d ../config/kitti00-02.yaml -v ../code/Vocabulary/ORBvoc.bin -c 3 -l info
```

## 5.3 Example on the TUM dataset
```shell
cd SwarmMap/bin/
./swarm_map -d ../config/fr2-large12.yaml -v ../code/Vocabulary/ORBvoc.bin -c 2 -l info
```

## 5.4 Standalone mode
- server
  ```shell
  # start the server first
  ./swarm_server --voc ../code/Vocabulary/ORBvoc.bin --log info --viewer 1 --mapviewer 1 --dataset PATH_TO_DATASE_FILE
  ```

- client #1
  ```shell
  # start the first client
  ./swarm_client --voc ../code/Vocabulary/ORBvoc.bin --log info --viewer 1 --mapviewer 1 --dataset ../config/mh1.yaml
  ```

- client #2
  ```shell
    # start the second client
  ./swarm_client --voc ../code/Vocabulary/ORBvoc.bin --log info --viewer 1 --mapviewer 1 --dataset ../config/mh2.yaml
  ```


## 5.4 Output Files
- The maps generated by the clients are saved as `map-client-<client_id>.bin`
- The maps generated by the server are saved as `map-server-<client_id>.bin`
- The global map on the server is saved as `map-global.bin`
- The trajectory of the agents is saved as `KeyFrameTrajectory=CURRENT_TIME-<client_id>.txt` and the format is TUM (`<timestamp> <tx> <ty> <tz> <qx> <qy> <qz> <qw>`)
- Trajectories in *TUM format* can be directly evaluated using the [evo evaluation tool](https://github.com/MichaelGrupp/evo).

## 6. Acknowledgements
SwarmMap is based on the following repositories:
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [Dynamic_ORB_SLAM2](https://github.com/Horacehxw/Dynamic_ORB_SLAM2)
- [ORB-SLAM2-GPU2016-final](https://github.com/yunchih/ORB-SLAM2-GPU2016-final) (GPU version of ORB-SLAM2)
