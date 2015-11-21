import numpy as np
from pybrain.tools.xml.networkreader import NetworkReader

net = NetworkReader.readFrom('ik_net.xml')

i = np.arange(0, 1000)
print i
