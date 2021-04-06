#!/usr/bin/env python

import sys

from hippocampus_common.radio_rqt_plugin import RadioConfiguratorPlugin
from rqt_gui.main import Main

plugin = 'radio_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
