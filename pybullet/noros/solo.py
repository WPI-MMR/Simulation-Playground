import functools
import math
import util
import os

from typing import List, Tuple

from pinocchio import utils as se3util
from pinocchio import robot_wrapper as se3robot_wrapper
import pinocchio as se3


class BaseSolo(object):
  """The base solo class

  Args:
    model_path (str): The path to the root of the model folder
    robot_name (str): Name of the robot, typically the name of the urdf file
    yaml_config (str): Name of the yaml configuration
    motor_inertia (float): Motor innertia constant
    motor_gear_ration (float): Motor gear ratios

  Returns:
  """
  # PID gains
  kp: float = 5.0
  kd: float = 0.1
  ki: float = 0.0

  # The Kt constant of the motor [Nm/A]: tau = I * Kt.
  motor_torque_constant: float = 0.025

  # Control time period.
  control_period: float = 0.001
  dt: float = control_period

  # Maximum control one can send, here the control is the current.
  # MaxCurrent = 12 # Ampers
  max_current: float = 2.0
  max_control: float = max_current

  max_qref: float = math.pi

  base_link_name: str = 'base_link'
  end_effector_names: List[str] = ['HL_ANKLE', 'HR_ANKLE', 'FL_ANKLE', 
                                   'FR_ANKLE']

  def __init__(self, model_path: str, robot_name: str, yaml_config: str,
               motor_inertia: float, motor_gear_ration: float):
    self.model_path = model_path
    self.urdf_path, self.mesh_path, self.yaml_path = util.model_to_subpaths(
      model_path, robot_name, yaml_config)

    self.motor_inertia = motor_inertia
    self.motor_gear_ration = motor_gear_ration

    self.mass: float = None
    self.base_name: str = None
    self.nb_joints: int = None
    self.map_joint_name_to_id: Dict[str, int] = None
    self.map_joint_limits: Dict[int, Tuple[float, float]] = None

  @property
  def max_torque(self):
   return self.motor_torque_constant * self.max_current

  def _build_robot(self):
    """_build_robot Create a pinocchio/pybullet robot wrapper """
    # Rebuild the robot wrapper instead of using an existing model to also load
    # the visuals
    print('mesh path')
    print(self.mesh_path)

    robot = se3robot_wrapper.RobotWrapper.BuildFromURDF(
      self.urdf_path, self.mesh_path, se3.JointModelFreeFlyer())

    robot.rotorInertia[6:] = self.motor_inertia
    robot.rotorGearRatio[6:] = self.motor_gear_ration

    self.mass = np.sum([i.mass for i in robot.inertias])
    self.base_name = robot.frames[2].name

    # The number of motors, here they are the same as there are only revolute
    # joints.
    self.nb_joints = robot.nv - 6

    self.map_joint_name_to_id = {}
    self.map_joint_limits = {}
    for i, (name, lower, upper) in enumerate(zip(robot.names[1:],
                                                 robot.lowerPositionLimit,
                                                 robot.upperPositionLimit)):
      self.map_joint_name_to_id[name] = i
      self.map_joint_limits[i] = (float(lower), float(upper))
    
    return robot


class Solo8Vanilla(BaseSolo):
  robot_name: str = 'solo'
  yaml_config: str = 'dmg_parameters_solo8.yaml'

  # The inertia of a single blmc_motor.
  motor_inertia: float = 0.0000045

  # The motor gear ratio.
  motor_gear_ration: int = 9.

  # Define the initial state.
  initial_configuration = [0.2,0,0.4, 0,0,0,1] + 4*[0.8,-1.6]
  initial_velocity = (8+6)*[0,]

  def __init__(self, model_path: str):
    self.q0 = None
    self.v0 = None
    self.a0 = None

    super().__init__(model_path, self.robot_name, self.yaml_config, 
                   self.motor_inertia, self.motor_gear_ration)

  def build_robot_wrapper(self):
    robot = self._build_robot()

    self.q0 = se3util.zero(robot.nq)
    self.q0[:] = self.initial_configuration
    self.v0 = se3util.zero(robot.nv)
    self.a0 = se3util.zero(robot.nv)

    return robot