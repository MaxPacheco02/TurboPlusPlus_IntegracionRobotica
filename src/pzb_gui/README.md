# Run the GUI following these steps
```Shell
# In first terminal
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# In another terminal
cd ~/TurboPlusPlus_IntegracionRobotica/src/pzb_gui
python3 -m http.server 8000

# Open in your browser
http://localhost:8000/pzb.html
```