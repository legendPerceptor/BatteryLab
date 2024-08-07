# BatteryLab
We aim to build an autonomous laboratory for building coin-cell batteries.

## Known Issues

### 1. prebuilt cv_bridge package is not compatible with Numpy 2.0

We added a submodule to build the cv_bridge from source to fix this problem. You need to install the Boost Python library to build this package. The required packages can be installed by the following commands.

```bash
sudo apt update
sudo apt install libboost-all-dev
```

And because we have a submodule in the repo. You may need to fetch the submodules before running `colcon build`.

```bash
git submodule update --init --recursive
```