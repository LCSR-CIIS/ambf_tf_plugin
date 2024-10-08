# ambf_tf_plugin
AMBF plugin for transformations. This plugin directly apply transformation to the rigidbody in AMBF. The predefined transformation can be applied only at the initialization phase (`INITIAL`) all the time (`FIXED`), or using rostopic (`ROS`).

## 1. Installation Instructions:
Let's call the absolute location of this package as **<plugin_path>**. E.g. if you cloned this repo in your home folder, **<plugin_path>** = `~/ambf_tf_plugin/` OR `/home/<username>/ambf_tf_plugin`.

### 1.1 clone and build the repository
```bash
git clone git@github.com:LCSR-CIIS/ambf_tf_plugin.git
cd ambf_tf_plugin
mkdir build && cd 
make
```

## 2. How to run the plugin
```bash
ambf_simulator --plugins <plugin_path>/build/libambf_tf_plugin.so --tf_list example/tf_list_example.yaml
```

In your `tf_list.yaml`, you can add as many transformations in the following format:
```tf_list_example.yaml
transformations:
  - TF World-PegBoard
  - TF Cube-Sphere
  - TF ROS-Test

TF World-PegBoard:
  parent: World # World: AMBF world origin
  child: BODY Puzzle_Board
  type: INITIAL # This transformation will be applied only once at the initialization phase
  transformation: # 4x4 Transformation matrix
  transformation: [
  [ 1.0, 0.0, 0.0, 0.0], 
  [ 0.0, 1.0, 0.0, 0.0],
  [ 0.0, 0.0, 1.0, 0.5],
  [ 0.0, 0.0, 0.0, 1.0]]

  TF Cube-Sphere:
  parent: Cube # World: AMBF world origin or Name of rigidBody
  child: Sphere
  type: FIXED  # This transformation will be applied all the time
  transformation: 
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {r: 0.0, p: 0.0, y: 0.0}

TF ROS-Test:
  parent: Cube
  child: ROS-sphere
  type: ROS  # The transformation will be received through ROS.
  rostopic name: /test_topic # Name of the rostopic (`PoseStampedPtr`)
```

The TF type can be 
- `INITIAL`: Predefined transformation can be applied only at the initialization phase 
- `FIXED`: Predefined transformation can be applied ALL the time
- `ROS`: Transformation from ROS topic will be applied

Predefined transformation can be written in 4x4 matrix or in `position`/`orientation` format:
```
position: {x: 0.0, y:0.0, z:0.0}, orientation: {r: 0.0, p: 0.0, y:0.0}
```


## Known problem
- If the object is `static`, the object will not move. Add mass and increase the friction/damping if needed.