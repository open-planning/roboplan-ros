# roboplan-ros

ROS 2 wrappers for the [roboplan](https://github.com/open-planning/roboplan) package.

## Development Setup

The upstream `roboplan` package is included as a submodule, be sure to clone it recursively.

```bash
git clone --recursive https://github.com/open-planning/roboplan-ros
```

ROS and the required dependencies are bundled together with [pixi](https://pixi.sh/latest/) and [robostack](https://robostack.github.io).
To set up, build, and test the environment:

```bash
# Setup the environment
pixi run setup-colcon

# Build everything
pixi run build

# Run tests
pixi run tests
```

Once the workspace is built, it can be used as a "normal" ROS 2 workspace.
That is, the workspace can be sourced and used as expected.

```bash
# Can also build with colcon
colcon build

# Or test,
colcon test
```
