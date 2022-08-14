# opensprinkler-pi

Fork of the [OpenSprinkler firmware](https://github.com/OpenSprinkler/OpenSprinkler-Firmware) which only supports
the [OpenSprinkler Pi](https://opensprinkler.com/product/opensprinkler-pi/).

The goal is to modernize and clean up the code, while fixing bugs without
having to support tech debt from the other platforms.

## Setup

1. Install the dependencies `mosquitto` and `libgpiod`.
2. Build the binary with `cmake`:

```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
```

This will create a `opensprinkler` binary in the `build` directory. Just
run it with a user that has access to the GPIO ports (like root, but
hopefully not root :)).
