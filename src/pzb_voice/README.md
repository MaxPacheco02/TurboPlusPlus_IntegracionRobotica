# How to use

For simplicity recording the audio and the pre-processing is done outside the node. We recomend conda for the environment. 

```
cd src/pzb_voice/scripts/VoiceLibs/
python live_testing.py
```

And from the `src/pzb_voice/scripts/` run the python node

```
cd src/pzb_voice/scripts/
python3 pzb_voice_node.py
```

This is because it needs to access the file `command.json` which is constantly overwritten. And if run from `src/` with `ros2 run` it will be run from `install/`, it won't be able to find the file.

# Libraries for running VoiceLibs

Found in scripts/requirements.txt


# Best words for live testing
```
    begin
    left
    down
    backward
```

# Encoding for ROS2
```
    "backward" : 0,
    "begin" : 1,
    "down" : 2,
    "finish" : 3,
    "forward" : 4,
    "left" : 5,
    "right" : 6,
    "rotate" : 7,
    "stop" : 8,
    "up" : 9
```
