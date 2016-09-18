import unittest
import os

from utils.rbag.poses import PosesBagLoader, PosesBagSaver
from utils.pdt.trajectory import CartTrajectory

class Test(unittest.TestCase):
    
    def test_loadPoses(self):
        loader = PosesBagLoader()        
        loader.read(os.path.dirname(os.path.realpath(__file__)) + "/cart-trajectory.bag")
        
        self.assertIsInstance(loader.data(), CartTrajectory)  
        
    def test_savePoses(self):
        saver = PosesBagSaver()
        saver.addPose([1., 2., 3., 4., 5., 6.])
        saver.addPose([2., 4., 6., 8., 10., 12])
        
        saver.write(os.path.dirname(os.path.realpath(__file__)) + "/cart-trajectory.temp1.bag")

    def test_saveTrajectory(self):
        cartTraj = CartTrajectory()
        cartTraj.addPose([1., 2., 3., 4., 5., 6.])
        cartTraj.addPose([6., 5., 4., 3., 2., 1])                
        
        saver = PosesBagSaver()
        saver.setTrajectory( cartTraj )
        saver.write(os.path.dirname(os.path.realpath(__file__)) + "/cart-trajectory.temp2.bag")

if __name__ == "__main__":
    unittest.main()
