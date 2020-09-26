import argparse
import os


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
                        type=str, default='test')

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
    return parser.parse_args()