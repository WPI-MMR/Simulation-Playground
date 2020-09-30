import argparse
import pathlib
import os

from typing import Tuple


def model_to_subpaths(model_path:str, robot_name:str, yaml_config:str,
                      urdf_folder: str = 'urdf', mesh_folder: str = 'meshes',
                      yaml_folder: str = 'yaml') -> Tuple[str, str, str]:
  urdf_path = os.path.join(model_path, '{}/{}.urdf'.format(urdf_folder,
                                                           robot_name))
  mesh_path = os.path.join(model_path, mesh_folder)
  yaml_path = os.path.join(model_path, '{}/{}'.format(yaml_folder, yaml_config))

  return urdf_path, mesh_path, yaml_path

class Config(object):
  """ Argument parser + configuration class.
  
  Attributes:
    init: Whether or not the class has already been initialized. Used for 
      advanced configuration inheritance.
  """
  def __init__(self):
    self.init: bool = False

  def _init(self, parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """_init Add base configuration onto the parser

    Args:
      parser (argparse.ArgumentParser): The existing parser

    Returns:
      argparse.ArgumentParser: The parser with the new options appended to it.
    """
    parser.add_argument('--model_root', help='root to the model meshes',
                        type=str, default='../../model/')
    parser.add_argument('--robot_name', help='The name of the robot, also the '
                        'name of the urdf file', type=str, default='solo')
    parser.add_argument('--yaml_config', help='The configuration of the robot.'
                        'Note that you NEED TO INCLUDE THE FILE EXTENSION',
                        type=str, default='dgm_parameters_solo8.yaml')

    self.init = True
    return parser
  
  def parse(self) -> argparse.Namespace:
    """Parse the commandline args for the class's options.

    Returns:
      argparse.Namespace: The configuration object
    """
    if not self.init:
      parser = argparse.ArgumentParser()
      parser = self._init(parser)
    
    self.parser = parser
    opt = parser.parse_args()

    return opt