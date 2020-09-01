
## Build


```
cd docker
./build.bash ignition-citadel ./Dockerfile.ignition
```

## Run image

```
 ./run.bash ignition-citadel PATH_TO/world_exporter
```

## Compile code

(once inside the docker container)

```
cd /home/developer/workspaces/src/
mkdir build
cd build
cmake ..
make
```

## Run code

### Set LD_LIBRARY_PATH

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.
```

### RUN

```
ign gazebo ../shapes_test.sdf -v 4
```
