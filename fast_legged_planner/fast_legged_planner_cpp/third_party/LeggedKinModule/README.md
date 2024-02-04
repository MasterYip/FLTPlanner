# LeggedKinModule

A kinematic module for legged robots which have leg-symmetry using Pinocchio &amp; IKFast.

## Positioning

- This kinematic module is for **Position Ctrl**(Trajectory tracking).
- For generality reason,this module supports all legged robots (hexapod) that has **leg symmetry**.
- Hope it is faster than using whole robot urdf which should **update every frame when calculating kinematics**
- This Module might **NOT help with MPC controlling** (OCS2 MPC don't use IK and has its pinocchio interface)

## Usage

1. Prepare Single Leg Urdf (for pinocchio FK)

   Prepare a urdf file that contains only one leg, put it in `config` folder.

2. Generate IKFast

   Use [IKFast_warpper](https://github.com/MasterYip/IKFast_warpper) to generate cpp library named `ikfast_leg_lib` with IK algorithm in namespace `ikfast_leg`.

   Then put it in `config` folder.

   > HexLab repo: [IKFast_warpper](https://github.com/HITSME-HexLab/IKFast_warpper)

3. Write your robot own kinematic module

   Write your own kinematic module in `config` folder. You can refer to [`config/elspider_air_kin.h`](config/elspider_air_kin.h).

> example is provided in `config` folder.

### Config Explanation

Config parameters are defined in [`core/single_leg_kin.h`](core/single_leg_kin.h).

```c++
std::string end_effector_name_ = "RF_FOOT";
// Origin Calib: move the origin of the leg to the origin of the base frame
Eigen::Vector3d origin_calib_ = Eigen::Vector3d::Zero(); // leg origin in base frame
// Installation Offsets (in order: mirror, rot, pos )
Eigen::Vector3d mirror_offset_ = Eigen::Vector3d::Ones();  // Mirror operation
Eigen::Matrix3d rot_offset_ = Eigen::Matrix3d::Identity(); // Rotation operation
Eigen::Vector3d pos_offset_ = Eigen::Vector3d::Zero();     // Translation operation
// Joint Directions (default: [1, 1, 1], decide before joint limits)
Eigen::Vector3d joint_directions_ = Eigen::Vector3d::Ones();
// Joint limits (default: [-pi, pi], order: [min, max, min, max, min, max])
Eigen::VectorXd joint_limits_ = Eigen::VectorXd::Zero(6);
// IKFast Approx point (usually the middle point of the joint limits)
Eigen::Vector3d ik_approx_point_ = Eigen::Vector3d::Zero();
```

- `end_effector_name_`: The name of the end effector frame in the urdf.
- `origin_calib_`: Sometimes in the urdf, the origin of the leg is not the origin of the robot. This parameter is used to calibrate the origin of the robot to the origin of the world frame.
- Installation Offsets: We want to **use only single leg urdf for all legs**, so we need to use installation offsets to transform the template leg to the expected position. There are three operations to move the leg in order: mirror, rotation, translation.
  - `mirror_offset_`: Do mirror operation to the leg, usually used for the left and right legs.
  - `rot_offset_`: Do rotation operation to the leg.
  - `pos_offset_`: Do translation operation to the leg, usually used for the front and back legs.
- `joint_directions_`: Redifine the joint directions, this might be useful for high level interface.
- `joint_limits_`: Joint limits of the leg, after `joint_directions_` is applied, default is [-pi, pi].
- `ik_approx_point_`: The point used for IKFast approximation when there are no solution, usually the foot position when joints are in the middle of the joint limits.

## TODO

- [x] Do dichotomy search outside of IKFast_warpper
- [x] Consider for code reusability
- [ ] Add Jacobian & Hessian(Jacobian time derivative) support

## Note

- **`pinocchio` will conflict with `boost`**, which should be included after `pinocchio`

  > USE_PINOCCHIO macro is reserved for setting pinocchio & boost include order

## Contribution

To ensure the format and security of our commits, we've defined some git hooks. We use [husky](https://github.com/typicode/husky) to manage these git hooks. First, you need to install husky, and then enable the git hooks with ```husky install```. And you can use ```pre-commit run -a``` to test [pre-commit](https://pre-commit.com/) hooks.

Code formatting utilizes Clang-Format with a modified Microsoft style, which is described by the .clang-format file.

Static checking involves:

- clang-tidy, as described by the .clang-tidy file.
