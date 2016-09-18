# =======================================================================================   
# More DMPs, a set of classes to play and learn with DMPs.
# dgerod@xyz-lab.org.es - 2015/16
# =======================================================================================

import numpy as np

import tf
import geometry_msgs.msg as rosmsg
import utils.pdt.trajectory as pdt
from common import BagLoader, BagSaver

# ---------------------------------------------------------------------------------------

class PosesBagLoader(BagLoader):
    """
    It reads 'geometry_msgs/PoseStamped' messages from a ROS bag. And the messages
    are stored as a 'np.array' of [num_samples x 6].

    All the messages are read from only one topic, and it is '/pose' by
    default. However, it could be changed accessing 'TopicName' attribute.
    """
    def __init__(self, topicName="/pose"):
        super(PosesBagLoader,self).__init__(topicName, "")

    def poses(self):
        """It returns stored poses as a 'np.array' of [num samples x 6]."""
        return self.Data.poses()

    def _readMessages(self, Bag):
        # Loads messages from an open bag and stored them as a 'CartTrajectory'.

        poses = []
        for topic, msg, t in Bag.read_messages(self._TopicName):

            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

            x = msg.pose.orientation.x; y = msg.pose.orientation.y
            z = msg.pose.orientation.z; w = msg.pose.orientation.w
            orientation = np.array(tf.transformations.euler_from_quaternion([x,y,z,w]))

            poses.append( np.concatenate([position, orientation]) )

        Traj = pdt.CartTrajectory()
        Traj.setPoseCollection( poses )
        return Traj

# ---------------------------------------------------------------------------------------

class PosesBagSaver(BagSaver):
    """
    It writes poses in a ROS bag as 'geometry_msgs/PoseStamped'. The messages are
    previously stored as a 'np.array' of [num_samples x 6] in this class.

    All the messages are write to only one topic, and it is '/pose' by
    default. However, it could be changed accessing 'TopicName' attribute.
    """

    def __init__(self, topicName="/pose"):
        super(PosesBagSaver,self).__init__(topicName, "")
        self.Data = pdt.CartTrajectory()

    def addPose(self, Pose):
        """Add a 'np.array' of [num samples x 6] to be stored in the bag."""
        self.Data.addPose(Pose)

    def setTrajectory(self, Traj):
        self.Data.clean()
        for pose in Traj.poses():
            self.Data.addPose(pose)

    def poses(self):
        """It returns stored poses as a 'np.array' of [num samples x 6]."""
        return np.array(self.Data.poses())

    def _writeMessages(self, Bag, Data):
        # Create a a list of messages from poses [num samples x 6 dof]
        # That's one message per position

        messages = []
        for pose in Data.poses():

            msg = rosmsg.PoseStamped()
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            q = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
            msg.pose.orientation = rosmsg.Quaternion(*q)

            messages.append(msg)

        for msg in messages:
            Bag.write(self._TopicName, msg)

# ---------------------------------------------------------------------------------------

def load_poses(BagName):
    """
    It reads 'geometry_msgs/PoseStamped' messages from a ROS bag. All the messages are
    read from only one topic, and it is '/pose'.

    It returns a 'np.array' of [num_samples x 6].

    Parameters
    ----------
    BagName : str
              File name of the bag.
    Returns
    -------
    Poses : np.array [num_samples x 6]
            Data loaded from the bag
    """
    bagLoader = PosesBagLoader()
    bagLoader.read(BagName)
    Poses = bagLoader.poses()
    return Poses

def save_poses(Poses, BagName):
    """
    It writes joints in a ROS bag as 'geometry_msgs/PoseStamped'. All the messages are
    write to only one topic, and it is '/pose'.

    Parameters
    ----------
    Poses   : np.array [num_samples x 6]
              Data to be stored in the bag.
    BagName : str
              File name of the bag.
    """
    bagSaver = PosesBagSaver()
    for p in Poses:
        bagSaver.addPose(p)
    bagSaver.write(BagName)

# =======================================================================================
