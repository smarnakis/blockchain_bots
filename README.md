# blockchain_bots

An application of a MRS communicating through blockchain. 

## Prereqs:

- Docker, Docker-compose
- Ros2 (foxy)
- Webots simulator

## Usage:
> Download and install the [sawtooth-sdk-python](https://github.com/hyperledger/sawtooth-sdk-python) and transfer the containts of [blockchain_bots/sawtooth-sdk-python](https://github.com/smarnakis/blockchain_bots/tree/master/sawtooth-sdk-python) overwriting the docker-compose.yaml file. (Might be a good idea to familiarize with [hyperledger sawtooth](https://sawtooth.hyperledger.org/) first)

> Clone or copy this repo in the src folder of your ros2 workspace:

```
cd ~/path_to_your_ros2_workspace/src
```

> Build the project at the root of your ros2 workspace:

```
cd ~/path_to_your_ros2_workspace
colcon build
```
> Open 2 new terminals and source the overlay:

```
cd ~/path_to_your_ros2_workspace
. install/setup.bash 
```

> On the first one, run the *sawtooth_bridge* node responsible for the 2 services for connecting and sending commands to sawtooth:

```
ros2 run sawtooth_bridge service
```

> On the second one, launch the simulation:

```
ros2 launch blockchain_bots_sim robot_launch.py
```

