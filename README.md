# Interface RS485

This module is S.O.N.I.A's comunication interface for RS485 based on [ROS](http://www.ros.org/)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

```bash
$ git clone https://github.com/sonia-auv/interface_RS485
```

### Prerequisites

You must install S.O.N.I.A's ROS repostitories to use this module.

S.O.N.I.A's installation instruction are available at [SONIA's Installation](https://sonia-auv.readthedocs.io/user/installation/)

## General informations

### Input/Output

Takes bytes from the hardware and traduce it for the software and vice versa. [Message structure](https://github.com/sonia-auv/interface_RS485/blob/develop/msg/SendRS485Msg.msg)

### Algorithms

Multi thread parser, reader and writer on the serial bus.

## Running the tests

Not implemented yet

## Deployment

Must be deployed with the sonia custom command sas

## Built With

* [ROS](http://www.ros.org/) - ROS Robotic Operating System


## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## Authors


* **Lucas Mongrain** - *Initial work* - [0x72d0](https://github.com/0x72D0)
* **Tom Robitaille** - *Documentation* - [Tommylas](https://github.com/Tommylas)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the GNU GPL V3 License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* **Chris Lietchi** - *Initial Python code* -
