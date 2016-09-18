import unittest
import os

from utils.rbag.joints import JointsBagLoader, JointsBagSaver
from utils.pdt.trajectory import PtpTrajectory

class Test(unittest.TestCase):
   
    def test_loadJoints(self):
        loader = JointsBagLoader()
		
        loader.read(os.path.dirname(os.path.realpath(__file__)) + "/ptp-trajectory.bag")
        self.assertIsInstance(loader.data(), PtpTrajectory)        
        
    def test_saveJoints(self):
        saver = JointsBagSaver()
        saver.addJointStates([1., 2., 3., 4., 5., 6., 7.])
        saver.addJointStates([2., 4., 6., 8., 10., 12., 14.])
        
        saver.write(os.path.dirname(os.path.realpath(__file__)) + "/ptp-trajectory.temp1.bag")
    
    def test_saveTrajectory(self):
        ptpTraj = PtpTrajectory()
        ptpTraj.addJoints([1., 2., 3., 4., 5., 6., 7.])
        ptpTraj.addJoints([7., 6., 5., 4., 3., 2., 1])                
        
        saver = JointsBagSaver()
        saver.setTrajectory( ptpTraj )
        saver.write(os.path.dirname(os.path.realpath(__file__)) + "/ptp-trajectory.temp2.bag")

if __name__ == "__main__":
    unittest.main()
