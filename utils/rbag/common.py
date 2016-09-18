# =======================================================================================   
# More DMPs, a set of classes to play and learn with DMPs.
# dgerod@xyz-lab.org.es - 2015/16
# =======================================================================================

import os
import rosbag
import std_msgs.msg as rosmsg

# ---------------------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------------------

class BagLoader(object):

    def __init__(self, TopicName, Info):
        self._TopicName = TopicName
        self.Info = Info
        self.Data = None

    def _prepFileName(self, BagName):
        # Check if the '.bag' is already in the name, and if it not
        # add to the extension to the bag name."""
        fileName, fileExt = os.path.splitext(BagName)
        if fileExt != ".bag":
            return  BagName + ".bag"
        else:
            return BagName

    def _readDescription(self, Bag):
        # Read the 'info' message where description of the bag is stored.
        # Only 1 information message is allowed.
        Info = ""
        for topic, msg, t in Bag.read_messages("info"):
            Info = msg.data
        return Info

    def _checkTopicExists(self, Bag):

        for topic, message, t in Bag.read_messages(self._TopicName):
            pass

        try: message
        except NameError:
            raise IOError("The [%s] topic does not exist in the bag." % self._TopicName)

    def _readMessages(self, Bag):
        """Read the messages from the bag and transform them."""
        return NotImplemented

    def read(self, BagName):
        """Open the bag and read the messages."""
        bag = rosbag.Bag(self._prepFileName(BagName), "r")
        self.Info = self._readDescription(bag)
        self._checkTopicExists(bag)
        self.Data = self._readMessages(bag)
        bag.close()
        return self.Data

    def data(self):
        return self.Data

# ----------------------------------------------------------------------------------------

class MsgImporter(object):
    def parse(self, Messages):
        return NotImplemented

class BagLoaderWithMsgImporter(BagLoader):

    def __init__(self, TopicName, MsgImporter):
        """Create an object adding an importer."""
        super(BagLoaderWithMsgImporter,self).__init__(TopicName, "")
        self._MsgImporter = MsgImporter

    def _readMessages(self, Bag):
        # Read all messages from the bag.
        messages = []
        for topic, msg, t in Bag.read_messages(self._TopicName):
            messages.append(msg)
        # And tranform them using importer
        return self._MsgImporter.parse(messages)

# ---------------------------------------------------------------------------------------
# Savers
# ---------------------------------------------------------------------------------------

class BagSaver(object):

    def __init__(self, TopicName, Info):
        self._TopicName = TopicName
        self.Info = Info
        self.Data = None

    def _prepFileName(self, BagName) :
        # Check if the ".bag" is already in the name, and if it not
        # add to the extension to the bag name.
        fileName, fileExt = os.path.splitext(BagName)
        if fileExt != ".bag":
            return  BagName + ".bag"
        else:
            return BagName

    def _writeDescription(self, Bag, Info):
        # Write description of the bag.
        # Only 1 information message is allowed.
        if Info != "":
            metadata = rosmsg.String()
            metadata.data = self.Info
            Bag.write("info", metadata)

    def _writeMessages(self, Bag, Data):
        """Read the messages from the bag and transform them."""
        return NotImplemented

    def write(self, BagName):
        """Open the bag and read the messages."""
        bag = rosbag.Bag(self._prepFileName(BagName), "w")
        self._writeDescription(bag, self.Info)
        self._writeMessages(bag, self.Data)
        bag.close()

# ---------------------------------------------------------------------------------------

class MsgExporter(object):
    def parse(self, Messages):
        return NotImplemented

class BagSaverWithMsgExporter(BagSaver):

    def __init__(self, TopicName, MsgExporter):
        """Create an object adding an exporter."""
        super(BagLoaderWithMsgImporter,self).__init__(TopicName, "")
        self._MsgExporter = MsgExporter

    def _writeMessages(self, Bag, Data):
        # Tranform data using importer
        messages = self._MsgExporter.parse(Data)
        # And write all messages to the bag
        for msg in messages:
            Bag.write(self._TopicName, msg)

# =======================================================================================
