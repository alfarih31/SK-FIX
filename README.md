# SK-FIX
Thesis to have graduate in instrumentation physics, University of Indonesia

# Pre
## repo
1. python3
2. libturbojpeg

## pip
1. grpcio
2. grpcio-tools
3. dronekit-python
4. opencv-python
5. Flask
6. Flask-SocketIO

# Init
1. Make dependency in dep folder by ```catkin_make```
2. Initiate the environment: ```source dep/devel/setup.bash```

# Start
1. Run ```web_app/start.py``` and open browser at ```localhost:2000```
2. Edit ```.env``` to fit with your environment, such as ```BAUD```, ```MODEL```, ```LABEL```, and ```LIBTURBO```
3. Run ```main_local.py```