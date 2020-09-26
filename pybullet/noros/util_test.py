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


if __name__ == '__main__':
  unittest.main()