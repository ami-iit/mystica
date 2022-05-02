# Nomenclature
## Variables

To render the following LaTeX equations, we suggest to install the chrome extension [TeX All the Things](https://chrome.google.com/webstore/detail/tex-all-the-things/cbimabofgmfdkicghcadidpemeenbffn).

code Name | LaTeX | Description| Dimensions
-- | -- | -- | --
`pos_a_b` | $p^ {a}_ {a,b}$ | Position Vector| [3,1]
`pos_a_b_c` | $p^ {a}_ {b,c}$ | Position Vector| [3,1]
`rotm_a_b` | $R^ {a}_ {b}$ | Rotation Matrix| [3,3]
`tform_a_b` | $T^ {a}_ {b}$ | Homogeneous Matrix| [4,4]
`quat` | $q$ | Quaternion vector| [4,1]
`linkPos_0` | $p^ {0}_ {0,i}$ | link `i` position wrt inertial frame | [3,1]
`linkQuat_0` | $q^ {0}_ {i}$ | link `i` orientation (quaternion) | [4,1]
`linkEul_0` |   | link `i` orientation (euler angles) | [3,1]
`linkAngVel_0` | $\omega^ {0}_ {0,i}$ | link `i` angular velocity | [3,1]
`linkTwist_0` | $\begin{bmatrix} \dot{p}^ {0}_ {0,i} \\\\ \omega^ {0}_ {0,i} \end{bmatrix}$ | link `i` twist | [6,1]
`linkPosQuat_0` | $\begin{bmatrix} p^ {0}_ {0,i} \\\\ q^ {0}_ {i} \end{bmatrix}$ | link `i` position and orientation (quaternion) | [7,1]
`jAngVel_PJ` | $\omega^ {pj}_ {pj,cj}$ | joint `j` angular velocity | [3,1]
`mBodyTwist_0` | $v$ | vector links twists | [6*nLink,1]
`mBodyTwAcc_0` | $\dot{v}$ | vector links acceleration twists | [6*nLink,1]
`mBodyPosQuat_0` | $x$ | kinematic state wrt inertial frame | [7*nLink,1]
`mBodyVelQuat_0` | $\dot{x}$ | kinematic state derivative | [7*nLink,1]
`mBodyLinVel_0` | $\begin{bmatrix} \vdots \\\\ \dot{p}^ {0}_ {0,i} \\\\ \vdots \end{bmatrix}$ | vector links linear velocity | [3*nLink,1]
`mBodyAngVel_0` | $w = \begin{bmatrix} \vdots \\\\ \omega^ {0}_ {0,i} \\\\ \vdots \end{bmatrix}$ | vector links angular velocity | [3*nLink,1]
`jointsAngVel_PJ` | $\omega = \begin{bmatrix} \vdots \\\\ \omega^ {pj}_ {pj,cj} \\\\ \vdots \end{bmatrix}$ | vector of joints angular velocity | [3*nJoints,1]
`jointsAngVelBase_0` |                                                 | vector of joints angular velocity + base | [3*nJoints+6,1]
`mBodyPosVel_0` | $\chi = \begin{bmatrix} x \\\\ v \end{bmatrix}$ |  | [(7+6)*nLink,1] |
`mBodyPosVel_0_g` | $\chi_ g$ |  | [(7+6)*nLink,1] |
`mBodyVelAcc_0` | $\dot{\chi} = \begin{bmatrix} \dot{x} \\\\ \dot{v} \end{bmatrix}$ |  | [(7+6)*nLink,1] |
`motorsAngVel` | $\omega_ {act}= \begin{bmatrix} \vdots \\\\ \omega^ {pj}_ {pj,cj} \\\\ \vdots \end{bmatrix}$ | angular velocity actuated joints | [nMotors,1]
`motorsCurrent` | | | [nMotors,1] |
`Jc` | $J_ {c}$ | jacobian of constraints | [nConstraints,6*nLink] |
`dJc` | $\dot{J}_ {c}$ | jacobian of constraints time derivative | [nConstraints,6*nLink] |
`mBodyWrenchExt_0` | | | |
`jointsWrenchConstr_PJ` | | | [nConstraints,1] |

## Frames

| Frames | Description                          |
| ------ | ------------------------------------ |
| `0`    | inertial frame                       |
| `b`    | body frame        |
| `g`    | CoM frame        |
| `p`    | body frame located at the center of the parent link  |
| `c`    | body frame located at the center of the child link  |
| `pj`   | body frame located at the connection point (relative to joint `j`) of the parent link |
| `cj`   | body frame located at the connection point (relative to joint `j`) of the child link |

![Notes_210301_092248](https://user-images.githubusercontent.com/38210073/109470633-fd9fd100-7a6f-11eb-9deb-1edbe26d8c5b.jpg)

## Extra

| Name | Description      |
| --------- | ---------------- |
| `nLink`   | number of links  |
| `nJoints` | number of joints |
| `nMotors` | number of motors |
| `nConstraints` | number of constraints |
