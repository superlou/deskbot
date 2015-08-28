import os.path
import pandas as pd
from pybrain.structure import TanhLayer
from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.tools.xml.networkwriter import NetworkWriter
from pybrain.tools.xml.networkreader import NetworkReader


if __name__ == "__main__":
    df = pd.read_csv('training.csv')

    if os.path.isfile('ik_net.xml'):
        net = NetworkReader.readFrom('ik_net.xml')
    else:
        net = buildNetwork(3, 20, 3, hiddenclass=TanhLayer)

    ds = SupervisedDataSet(3, 3)

    for i, row in df.iterrows():
        training_input = (row.normal_x, row.normal_y, row.centroid_z)
        training_output = (row.theta0, row.theta1, row.theta2)
        ds.addSample(training_input, training_output)

    trainer = BackpropTrainer(net, ds)
    for i in range(1000):
        print "{0}: {1}".format(i, trainer.train())

    NetworkWriter.writeToFile(net, 'ik_net.xml')
