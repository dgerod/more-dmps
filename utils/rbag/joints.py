# =======================================================================================   
# More DMPs, a set of classes to play and learn with DMPs.
# dgerod@xyz-lab.org.es - 2015/16
# =======================================================================================

import numpy as np
import sensor_msgs.msg as rosmsg
import utils.pdt.trajectory as pdt
from common import BagLoader, BagSaver

# ---------------------------------------------------------------------------------------

class JointsBagLoader(BagLoader):
    """
    It reads 'sensor_msgs/JointState' messages from a ROS bag. And the messages
    are stored as a 'np.array' of [num_samples x num_joints].

    All the messages are read from only one topic, and it is '/joint_states' by
    default. However, it could be changed accessing 'TopicName' attribute.
    """

    def __init__(self, topicName="/joint_states"):
        super(JointsBagLoader,self).__init__(topicName, "")

    def joints(self):
        """It returns stored joints as a 'np.array' of [num samples x num joints]."""
        return self.Data.joints()

    def _readMessages(self, Bag):
        # Loads messages from an open bag and stored them as a 'PtpTrajectory'.

        jnts = []
        for topic, msg, t in Bag.read_messages(self._TopicName):
            jnts.append( np.array(msg.position) )

        Traj = pdt.PtpTrajectory()
        Traj.setJointsCollection( jnts )
        return Traj

# ---------------------------------------------------------------------------------------

class JointsBagSaver(BagSaver):
    """
    It writes joints in a ROS bag as 'sensor_msgs/JointState'. The messages are
    previously stored as a 'np.array' of [num_samples x num_joints] in this class.

    All the messages are write to only one topic, and it is '/joint_states' by
    default. However, it could be changed accessing 'TopicName' attribute.
    """

    def __init__(self, topicName="/joint_states"):
        super(JointsBagSaver,self).__init__(topicName, "")
        self.Data = pdt.PtpTrajectory()

    def addJointStates(self, JointStates):
        """Add a 'np.array' of [num samples x num joints] to be stored in the bag."""
        self.Data.addJoints(JointStates)

    def setTrajectory(self, Traj):
        self.Data.clean()
        for jointStates in Traj.joints():
            self.Data.addJoints(jointStates)

    def joints(self):
        """It returns stored joints as a 'np.array' of [num samples x num joints]."""
        return np.array(self.Data.joints())

    def _writeMessages(self, Bag, Data):
        # Create a a list of messages from joints [num_samples x num_joints].
        # That's one message per position.

        numJoints = self.Data.joints().shape[1]
        messages = []
        for pdx in range(len(Data.joints())):

            js = rosmsg.JointState()
            js.name = map(lambda x: "joint_%d" % (x+1), range(numJoints))
            #js.position = self.Data.Joints[pdx].tolist()
            js.position =self.joints()[pdx].tolist()
            messages.append(js)

        for msg in messages:
            Bag.write(self._TopicName, msg)

# ---------------------------------------------------------------------------------------

def load_joints(BagName):
    """
    It reads 'sensor_msgs/JointState' messages from a ROS bag. All the messages are
    read from only one topic, and it is '/joint_states'.

    It returns a 'np.array' of [num_samples x num_joints].

    Parameters
    ----------
    BagName : str
              File name of the bag.
    Returns
    -------
    Joints : np.array [num_samples x num_joints]
             Data loaded from the bag
    """
    bagLoader = JointsBagLoader()
    bagLoader.read(BagName)
    Joints = bagLoader.joints()
    return Joints

def save_joints(Joints, BagName):
    """
    It writes joints in a ROS bag as 'sensor_msgs/JointState'. All the messages are
    write to only one topic, and it is '/joint_states'.

    Parameters
    ----------
    Joints  : np.array [num_samples x num_joints]
              Data to be stored in the bag.
    BagName : str
              File name of the bag.
    """
    bagSaver = JointsBagSaver()
    for j in Joints:
        bagSaver.addJointStates(j)
    bagSaver.write(BagName)

# =======================================================================================
