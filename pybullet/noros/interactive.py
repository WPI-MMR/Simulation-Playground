import pybullet as p
import numpy as np
import time
import os

import util


dt = 1e-3


if __name__ == '__main__':
  # Parse client options
  config = util.Config().parse()

  urdf_path, mesh_folder, yaml_path = util.model_to_subpaths(
    config.model_root, config.robot_name, config.yaml_config)
  urdf_folder = os.path.dirname(urdf_path)

  # Set up client
  client = p.connect(p.GUI)
  p.setGravity(0, 0, -9.81)
  p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)

  # Load the environment
  plain_urdf = os.path.join(config.model_root, 
                            'urdf/plane_with_restitution.urdf')
  plain_id = p.loadURDF(plain_urdf)

  # Load the robot
  robotStartPos = [0., 0, 1.]
  robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

  robot_id = p.loadURDF(urdf_path, robotStartPos, robotStartOrientation, 
                        flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=False)


  # Setup interactive interface
  joint_params = []
  num_joints = p.getNumJoints(robot_id)
  p.setJointMotorControlArray(robot_id, np.arange(num_joints),
                              p.VELOCITY_CONTROL, forces=np.zeros(num_joints))

  for joint in range(num_joints):
    p.changeDynamics(robot_id, joint, linearDamping=.04, angularDamping=0.04, 
                     restitution=0.0, lateralFriction=0.5)
    joint_params.append(p.addUserDebugParameter(
      'Joint {}'.format(p.getJointInfo(robot_id, joint)[1].decode('UTF-8')), 
      -0.25, 0.25, 0.0))
  
  time.sleep(1)
  print('Press any key to stop the simulation')
  try:
    zero_gains = num_joints * [0.]

    while True:
      user_joints = [p.readUserDebugParameter(param)
                     for param in joint_params]
      p.setJointMotorControlArray(robot_id, np.arange(num_joints), 
                                  p.TORQUE_CONTROL, forces=user_joints, 
                                  positionGains=zero_gains, 
                                  velocityGains=zero_gains)

      p.stepSimulation()
      time.sleep(dt)
  except KeyboardInterrupt:
    pass