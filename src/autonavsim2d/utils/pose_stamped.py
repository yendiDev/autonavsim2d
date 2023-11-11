
class PoseStamped:

    def __init__(self, header, pose):
        self.header = header
        self.pose = pose

class Header:

    def __init__(self, stamp, frame_id):
        self.stamp = stamp
        self.frame_id = frame_id
