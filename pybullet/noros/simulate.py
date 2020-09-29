import pinocchio as se3
import pybullet as p
import numpy as np
import time
import os

from pinocchio_bullet_wrapper import PinBulletWrapper
import util


dt = 1e-3
root = os.getcwd()


class Solo8Robot(PinBulletWrapper):
  @staticmethod
  def initPhysicsClient():
    client = p.connect(p.GUI)
    p.setGravity(0,0, -9.81)
    p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
    return physicsClient

  def __init__(self, config, physicsClient=None):
    self.config = config
    self.physicsClient = physicsClient or self.initPhysicsClient()

    plain_urdf = os.path.join(self.config.model_root, 
                              'urdf/plane_with_restitution.urdf')
    self.plain_id = p.loadURDF(plain_urdf)

    # Load the robot
    robotStartPos = [0., 0, 0.40]
    robotStartOrientation = p.getQuaternionFromEuler([0,0,0])


if __name__ == '__main__':
  Solo8Robot()