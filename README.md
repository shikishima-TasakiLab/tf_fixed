# tf_fixed

ROS node that continuously outputs /tf at regular intervals. Parameters allow configuration changes at any time.

```mermaid
flowchart LR

p@{ shape: lean-r }
node(["tf_fixed"])
tf["/tf"]

p -. Set Parameters .-> node -- Broadcast --> tf
```

## Install

```bash
cd (your ros2 workspace)/src
git clone https://github.com/shikishima-TasakiLab/tf_fixed.git
cd ..
colcon build
source ./install/setup/bash
```
