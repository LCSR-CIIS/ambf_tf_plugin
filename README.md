# ambf_tf_plugin
AMBF plugin for transformations.

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
type: Fixed # {Fixed, Initial, ROS}ons:
- TF World-PegBoard

TF World-PegBoard:
  parent: World # World: AMBF world origin
  child: BODY Puzzle_Board
  type: INITIAL # {FIXED, INITIAL, ROS}
  transformation: # 4x4 Transformation matrix
  transformation: [
  [ 1.0, 0.0, 0.0, 0.0], 
  [ 0.0, 1.0, 0.0, 0.0],
  [ 0.0, 0.0, 1.0, 0.5],
  [ 0.0, 0.0, 0.0, 1.0]]

```