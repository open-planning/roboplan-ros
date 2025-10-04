# roboplan-ros

ROS 2 wrappers for the [roboplan](https://github.com/open-planning/roboplan) package.

## Development Setup

The upstream `roboplan` package is included as a submodule, be sure to clone it recursively, as it also has submodules.

```bash
git clone --recursive https://github.com/open-planning/roboplan-ros
```

### Pixi Development

ROS and the required dependencies are bundled together with [pixi](https://pixi.sh/latest/) and [robostack](https://robostack.github.io).
Different versions of ROS (humble, jazzy, and kilted) are supported with pixi environments.

For instance, to build for kilted:

```bash
# Setup the environment
pixi run -e kilted setup-colcon

# Build
pixi run -e kilted build

# Run tests, must manually run build prior to executing
pixi run -e kilted test

# Show tests results
pixi run -e kilted test-result
```

Once the workspace is built, it can be used as a "normal" ROS 2 workspace.
That is, the workspace can be sourced and used as expected.
However, not that the build/install directories are broken out by ROS distribution, so those arguments MUST be included when compiling.

```bash
# Activate the kilted environment
pixi shell -e kilted

# Build with the existing install dirs
colcon build --build-base build_${ROS_DISTRO} --install-base install_${ROS_DISTRO}

# Source it
source install_${ROS_DISTRO}/setup.bash

# Or test,
colcon test --build-base build_${ROS_DISTRO} --install-base install_${ROS_DISTRO}
```

The examples from the upstream repository should now be available, as well,

```bash
python3 roboplan/bindings/examples/example_scene.py
```

### Docker Development

A [Docker Compose](https://docs.docker.com/compose/) workflow is also provided by the [compose specification](./docker-compose.yaml).

To build and run,

```bash
# Build the image, defaults to jazzy but can be overridden by setting $ROS_DISTRO
docker compose build ros

# Start the container
docker compose up ros

# Attach to the running container
docker compose exec ros bash
```

This will drop the user into a pre-compiled colcon workspace with the source mounted in the image.
