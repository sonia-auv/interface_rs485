# interface_rs485
This module is S.O.N.I.A's comunication interface for RS485 based on ROS

![Docker Image CI - Master Branch](https://github.com/sonia-auv/interface_rs485/workflows/Docker%20Image%20CI%20-%20Master%20Branch/badge.svg)
![Docker Image CI - Develop Branch](https://github.com/sonia-auv/interface_rs485/workflows/Docker%20Image%20CI%20-%20Develop%20Branch/badge.svg?branch=develop)
![GitHub release (latest by date)](https://img.shields.io/github/v/release/sonia-auv/interface_rs485)
![Average time to resolve an issue](https://isitmaintained.com/badge/resolution/sonia-auv/interface_rs485.svg)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

Clone current project by using following command :
```bash
    git clone git@github.com:sonia-auv/interface_rs485.git
```

### Prerequisites

First and foremost to run the module you will need to have [docker](https://www.docker.com/get-started?utm_source=google&utm_medium=cpc&utm_campaign=getstarted&utm_content=sitelink&utm_term=getstarted&utm_budget=growth&gclid=CjwKCAjw57b3BRBlEiwA1Imytuv9VRFX5Z0INBaD3JJNSUmadgQh7ZYWTw_r-yFn2S4XjZTsLbNnnBoCPsIQAvD_BwE) installed.

To validate your installation of docker, simply type in

```
docker -v
```

If you receive an output in the likes of :
```
Docker version 20.10.14, build a224086
```

It means you have it installed. If not follow instructions on how to install it for your OS.

### Installing

A step by step series of examples that tell you how to get a development env running



```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

<!-- ## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
``` -->

## Deployment


### Run on AUVs

In compose : 

```  
interface_rs485:
    image: ghcr.io/sonia-auv/interface_rs485/interface_rs485:x86-perception-feature-simulation
    container_name: interface_rs485
    environment:
      - ROS_IP=${ADRESS_IP}
      - ROS_MASTER_URI=http://${ADRESS_IP}:11311
      - AUV=${AUV}
    network_mode: host
    privileged: true
    volumes:
     - /dev/RS485:/dev/RS485
    depends_on:
     - ros-master
```

### Local

Build the docker image :

```docker build -t interface_rs485:latest .```

Run the docker image :

```docker run -it interface_rs485:latest```

OR

Compose with dev board :

```  
interface_rs485:
    image: ghcr.io/sonia-auv/interface_rs485/interface_rs485:x86-perception-feature-simulation
    container_name: interface_rs485
    environment:
      - ROS_IP=${ADRESS_IP}
      - ROS_MASTER_URI=http://${ADRESS_IP}:11311
      - AUV=${AUV}
    network_mode: host
    privileged: true
    volumes:
     - /dev/ttyUSB0:/dev/ttyUSB0
    depends_on:
     - ros-master
```
OR 

Compose with simulation :

```  
interface_rs485:
    image: ghcr.io/sonia-auv/interface_rs485/interface_rs485:x86-perception-feature-simulation
    container_name: interface_rs485
    environment:
      - ROS_IP=${ADRESS_IP}
      - ROS_MASTER_URI=http://${ADRESS_IP}:11311
      - AUV=${AUV}
    network_mode: host
    privileged: true
    depends_on:
     - ros-master
    command:
      - roslaunch
      - --wait
      - interface_rs485
      - interface_rs485_sim.launch
```

You can choose the sub by using AUV=${LOCAL_AUV} and indicating the AUV you want to simulate in LOCAL_AUV in launch_local.sh.

## Built With

Add additional project dependencies

* [ROS](http://wiki.ros.org/) - ROS perceptionic framework


## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## License

This project is licensed under the GNU License - see the [LICENSE](LICENSE) file for details
