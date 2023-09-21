# dh_transform
This repository was developed to accomplish the proposed challenge of (...)

## Considerations
This solution was developed for a 2D manipulator composed of N rotation joints, N links and one end effector, as showed bellow
<div align="center">
  <img src="resources/images/manipulator.png" alt="N links manipulator">
</div>

## Nodes
### Encoder
This node is used to simulate a group of joint encoder's, publishing their angular position.

To run the node use
```bash
$ ros2 run dh_transform encoder
```

Now the server is ready to take requests on runtime to change the currant angular positions.
```bash
$ ros2 service call /update_angles dh_transform/srv/AnglesUpdate "{angles: [1.0, 1.0, 20.0]}"
```

*Note that the number of values passed (double) depend on the number of joint declared.*

### Pose Calculator
This class has the mathematical implementation of the forward kinematics operations. So, it takes the number of manipulator's links and their respective lengths. After this, it is possible to obtain the Denavit Hartenberg (DH) matrix that describe the manipulator.

## Tests
When implement new features or modifying the existing code, it is important to validate the expected code behaver both in the new implementation and in the existing features. So, to accomplish this, run the tests. First of all build the package.

```bash
$ colcon build
```
And run the tests
```bash
$ colcon test
```

Finally, inspect the results
```bash
$ colcon test-result --verbose
```

Or, in a compact form, run the three commands on a single line 
```bash
$ colcon build --packages-select dh_transform && colcon test --packages-select dh_transform --event-handler=console_direct+ && colcon test-result --verbose
```