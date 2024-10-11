<div align="center" style="margin-bottom: 10px;">
<a href="https://github.com/raultapia/asap">
<img width="500" src=".github/assets/asap.png" alt="logo">
</a>
</div>
<p align="justify" class="brief">
ASAP is a novel event handling framework that dynamically adapts the transmission of events to the processing algorithm, keeping the system responsiveness and preventing overflows. ASAP is composed of two adaptive mechanisms. The first one prevents event processing overflows by discarding an adaptive percentage of the incoming events. The second mechanism dynamically adapts the size of the event packages to reduce the delay between event generation and processing.
</p>

## ⚙️ Installation

Some dependencies are required:

-   [ROS](https://wiki.ros.org/ROS/Installation)
-   [dvs_msgs](https://github.com/ros-event-camera/dvs_msgs) (original repo [here](https://github.com/uzh-rpg/rpg_dvs_ros))
-   [openev](https://github.com/raultapia/openev)

Then, ASAP can be installed.

For ROS1:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/raultapia/asap
catkin build asap
```

For ROS2:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/raultapia/asap
colcon build --packages-up-to asap
```

## 🖥️ Usage
```bash
roslaunch asap asap.launch
```
or
```bash
ros2 launch asap asap.launch.py
```

## 📜 Citation

If you use this work in an academic context, please cite the following publications:

> R. Tapia, J.R. Martínez-de Dios, A. Gómez Eguíluz, A. Ollero
> **ASAP: Adaptive Transmission Scheme for Online Processing of Event-Based Algorithms**,
> Autonomous Robots, 2022.

```bibtex
@article{tapia2022asap,
  author={Tapia, R. and Martínez-de Dios, J.R. and Gómez Eguíluz, A. and Ollero, A.},
  journal={Autonomous Robots},
  title={{ASAP}: Adaptive Transmission Scheme for Online Processing of Event-Based Algorithms},
  year={2022},
  volume={46},
  pages={879–892},
  doi={10.1007/s10514-022-10051-y}
}
```

> R. Tapia, A. Gómez Eguíluz, J.R. Martínez-de Dios, A. Ollero
> **ASAP: Adaptive Scheme for Asynchronous Processing of Event-Based Vision Algorithms**,
> IEEE International Conference on Robotics and Automation. Workshop on Unconventional Sensors in Robotics, 2020.

```bibtex
@inproceedings{tapia2020asap,
  author={Tapia, R. and Gómez Eguíluz, A. and Martínez-de Dios, J.R. and Ollero, A.},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA) Workshop on Unconventional Sensors in Robotics},
  title={{ASAP}: Adaptive Scheme for Asynchronous Processing of Event-Based Vision Algorithms},
  year={2020},
  doi={10.5281/zenodo.3855412}
}
```

## 📝 License

Distributed under the GPLv3 License. See [`LICENSE`](https://github.com/raultapia/openev/tree/main/LICENSE) for more information.

## 📬 Contact

[Raul Tapia](https://raultapia.com) - raultapia@us.es
