import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/argon/Documents/RoboResearch/A2/install/bansal_object_follower'
