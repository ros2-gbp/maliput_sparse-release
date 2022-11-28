[![GCC](https://github.com/maliput/maliput_sparse/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/maliput_sparse/actions/workflows/build.yml)

# maliput_sparse

## Description

`maliput_sparse` is a convenient package that provides several helpers for creating a maliput backend that is expected to be built on top of waypoints without any analytical model of the surface.

By using the [builder API](https://maliput.readthedocs.io/en/latest/html/deps/maliput_sparse/html/builder_8h.html), the mathematical model is solved under the hood so the user doesn't have to dive into complex geometric calculations.

**Note**: For full information about Maliput please visit [Maliput Documentation](https://maliput.readthedocs.io/en/latest/index.html).

## API Documentation

Refer to [Maliput sparse's Online API Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput_sparse/html/index.html).

## Examples

[Getting Started](https://maliput.readthedocs.io/en/latest/getting_started.html) page is a good place for starting to see Maliput's capabilities and how to use a Maliput backend for getting a road network.

 - [maliput_osm](https://github.com/maliput/maliput_osm): This maliput backend works as example on how `maliput_sparse` can be used for easily creating a backend that is based on a format that uses waypoints for describing the roads.

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS, ROS2 Foxy.

### Binary Installation on Ubuntu

See [Installation Docs](https://maliput.readthedocs.io/en/latest/installation.html#binary-installation-on-ubuntu).

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/maliput_sparse.git
    ```

3. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```

4. Build the package
    ```sh
    colcon build --packages-up-to maliput_sparse
    ```

    **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
    ```sh
    colcon build --packages-select maliput_sparse --cmake-args " -DBUILD_DOCS=On"
    ```
    More info at [Building Documentation](https://maliput.readthedocs.io/en/latest/developer_guidelines.html#building-the-documentation).

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)


### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/maliput_sparse/blob/main/LICENSE)
