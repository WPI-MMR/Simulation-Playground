from unittest import mock
import unittest

import util
import sys


class TestUtils(unittest.TestCase):
  def test_configuration_options(self):
    options = {
      'model_root': '/test/',
    }

    options_list = list(zip(['--{}'.format(option) for option in options.keys()], 
                             options.values()))
    options_flat = [value for option in options_list for value in option]
    fake_argv = ['python'] + options_flat

    with mock.patch.object(sys, 'argv', fake_argv):
      parsed_dict = vars(util.Config().parse())

    self.assertTrue(options.items() <= parsed_dict.items())

  def test_model2subpaths_basic(self):
    model_root = '/model'
    urdf, mesh, yaml = util.model_to_subpaths(model_root, 'robot',
                                               'config.yaml')

    self.assertEqual(urdf, '/model/urdf/robot.urdf')
    self.assertEqual(mesh, '/model/meshes')
    self.assertEqual(yaml, '/model/yaml/config.yaml')

  def test_model2subpaths_nofiles(self):
    model_root = '/model'
    urdf, mesh, yaml = util.model_to_subpaths(model_root, None, None)

    self.assertEqual(urdf, '/model/urdf/None.urdf')
    self.assertEqual(mesh, '/model/meshes')
    self.assertEqual(yaml, '/model/yaml/None')

  def test_model2subpaths_custom_folders(self):
    model_root = '/model'
    urdf, mesh, yaml = util.model_to_subpaths(
      model_root, 'robot', 'config.yaml', urdf_folder='test_urdf',
      mesh_folder='test_mesh', yaml_folder='test_yaml')
    
    self.assertEqual(urdf, '/model/test_urdf/robot.urdf')
    self.assertEqual(mesh, '/model/test_mesh')
    self.assertEqual(yaml, '/model/test_yaml/config.yaml')


if __name__ == '__main__':
  unittest.main()