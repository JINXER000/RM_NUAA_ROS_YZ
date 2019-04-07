#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from os.path import realpath, dirname, join
from numPred.msg import out_msg
import keras as K

class numPredNode(object):
    def __init__(self):
        super(numPredNode,self).__init__()
        model_path="/home/nvidia/yzchen_ws/ros_ws/src/numPred/models/numberClassify_Redmodel_min.h5"

        print("loading model from"+model_path)
        rospy.init_node('num_predictor',anonymous=True)
        self._bridge = CvBridge()
#        self.model=K.models.load_model(model_path)
        self.pub_num=rospy.Publisher('/numPred/num',out_msg,queue_size=1)
        self.sub_image=rospy.Subscriber("/armor_detector/armor_roi",Image,self.img_callback)
        print("initialize done")
        rospy.spin()

    def img_callback(self, data):
        try:
            # Convert image to numpy array
            self.current_image = self._bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(e)

        num=self.pred_number(self.current_image)
        pred_out=out_msg()
        pred_out.num=num
        self.pub_num.publish(pred_out)

    def pred_number(self,img):
        testImg_a=np.array(img)
        num=7

        # pred=self.model.predict(testImg_a)
        # num = np.argmax(pred)
        return num

 
def main():
    """ main function
    """
    node = numPredNode()

if __name__ == '__main__':
    main()
