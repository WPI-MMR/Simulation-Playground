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

if __name__ == '__main__':
  unittest.main()