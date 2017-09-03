import rospy
from tf import TransformListener

class myNode:
    def __init__(self, *args):
        self.tf = TransformListener()

    def some_method(self):

        if self.tf.frameExists("/base_footprint") and self.tf.frameExists("/map"):
            t = self.tf.getLatestCommonTime("/base_footprint", "/map")
            position, quaternion = self.tf.lookupTransform("/base_footprint", "/map", t)
            print position, quaternion