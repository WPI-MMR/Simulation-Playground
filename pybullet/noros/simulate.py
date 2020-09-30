import pinocchio as se3
import pybullet as p
import numpy as np
import time
import os

from pinocchio_bullet_wrapper import PinBulletWrapper
import util
import solo


dt = 1e-3
root = os.getcwd()


class Solo8Robot(PinBulletWrapper):
  @staticmethod
  def initPhysicsClient():
    client = p.connect(p.GUI)
    p.setGravity(0,0, -9.81)
    p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
    return client

  def __init__(self, config, physicsClient=None):
    self.config = config
    self.physicsClient = physicsClient or self.initPhysicsClient()

    plain_urdf = os.path.join(self.config.model_root, 
                              'urdf/plane_with_restitution.urdf')
    self.plain_id = p.loadURDF(plain_urdf)

    # Load the robot
    robotStartPos = [0., 0, 0.40]
    robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

    urdf_path, mesh_folder, yaml_path = util.model_to_subpaths(
      self.config.model_root, self.config.robot_name, self.config.yaml_config)
    urdf_folder = os.path.dirname(urdf_path)

    self.robot_id = p.loadURDF(urdf_path, robotStartPos, robotStartOrientation,
                               flags=p.URDF_USE_INERTIA_FROM_FILE,
                               useFixedBase=False)
    p.getBasePositionAndOrientation(self.robot_id)

    num_joints = p.getNumJoints(self.robot_id)
    for joint in range(num_joints):
      p.changeDynamics(self.robot_id, joint, linearDamping=.04, 
                       angularDamping=0.04, restitution=0.0, 
                       lateralFriction=0.5
    
if __name__ == '__main__':
  client_opt = util.Config().parse()
  robot = Solo8Robot(client_opt)