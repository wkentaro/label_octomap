# label_octomap

Probabilistic 3D Multilabel Real-time Mapping for Multi-object Manipulation

<a href="https://www.youtube.com/watch?v=T-vtVQT9sgc&t=7s">
  <img src="http://i3.ytimg.com/vi/T-vtVQT9sgc/maxresdefault.jpg" width="50%" />
</a>


## Citation

```
@InProceedings{Wada:etal:IROS2017,
  author={Kentaro Wada and Kei Okada and Masayuki Inaba},
  title={Probabilistic 3D Multilabel Real-time Mapping for Multi-object Manipulation},
  booktitle={Proceedings of the IEEE-RAS International Conference on Robotics and Systems (IROS)},
  year={2017},
}
```


## Installation

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

wstool init src https://raw.githubusercontent.com/wkentaro/hrp2_apc/master/rosinstall
rosdep install --from-path . -r -y -i
catkin build
```


## Usage

Pick the backward object in a shelf bin.

```bash
roslaunch label_octomap pick_backward_object.launch
```


## Private

- Paper: https://github.com/wkentaro/iros2017-label-octomap-paper
- Data: https://drive.google.com/drive/u/2/folders/0B9P1L--7Wd2vOG13SnBMaW04NE0
