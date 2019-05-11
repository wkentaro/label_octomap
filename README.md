<div align="center">
  <h1>label_octomap</h1>

  <h4>
    Probabilistic 3D Multilabel Real-time Mapping for Multi-object Manipulation (IROS2017)
  </h4>
  <p>
    <a href="https://ieeexplore.ieee.org/document/8206394">Paper (IEEE)</a>
    |
    <a href="https://drive.google.com/open?id=1frqieyHiQBqpr1e9mWPrzfaX8FbpLcuX">Paper (Gdrive)</a>
    |
    <a href="https://www.youtube.com/watch?v=T-vtVQT9sgc">Video</a>
    |
    <a href="https://github.com/wkentaro/label_octomap">Code</a>
    |
    <a href="https://drive.google.com/open?id=1c5W-nUooUo_9hMT0ei-34t3_lOhA3lZs">Slides</a>
  </p>

  <a href="https://www.youtube.com/watch?v=T-vtVQT9sgc">
    <img src=".readme/20170306_iros2017.gif" />
  </a>
</div>


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

```bash
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


<!--
## Private

- Paper: https://github.com/wkentaro/iros2017-label-octomap-paper
- Data: https://drive.google.com/drive/u/2/folders/0B9P1L--7Wd2vOG13SnBMaW04NE0
-->
