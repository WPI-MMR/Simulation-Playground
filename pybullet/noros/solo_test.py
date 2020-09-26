import unittest
import os

import solo


class TestSoloConfig(unittest.TestCase):
  def test_file_resolution(self):
    b = solo.BaseSolo('/base/path/', 'robot', 'config.yaml', 0.0, 1)

    self.assertEqual(b.model_path, '/base/path/')
    self.assertEqual(b.urdf_path, '/base/path/urdf/robot.urdf')
    self.assertEqual(b.mesh_path, '/base/path/meshes')
    self.assertEqual(b.yaml_path, '/base/path/yaml/config.yaml')

  def test_config_inheritance(self):
    solo8 = solo.Solo8Vanilla('/base/path')

    with self.subTest('no modification'):
      self.assertEqual(solo8.motor_inertia, 0.0000045)
      self.assertEqual(solo8.motor_gear_ration, 9)
    
    with self.subTest('instance modification'):
      solo8.motor_gear_ration = 100
      self.assertEqual(solo8.motor_gear_ration, 100)
      self.assertEqual(solo.Solo8Vanilla.motor_gear_ration, 9)

if __name__ == '__main__':
  unittest.main()