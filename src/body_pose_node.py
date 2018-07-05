#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Randoms
#


import os
import sys
import time

import numpy as np
import rospy
from body_pose.msg import BodyInfo
from body_pose.srv import BodyPose, BodyPoseResponse
from cv_bridge import CvBridge, CvBridgeError
from scipy.misc import imread, imsave

from config import load_config
from dataset.factory import create as create_dataset
from dataset.pose_dataset import data_to_input
from multiperson.detections import extract_detections
from multiperson.predict import (SpatialModel, eval_graph,
                                 get_person_conf_multicut)
from nnet import predict
from util import visualize

# setup tensorflow date
current_path = os.path.dirname(__file__)
os.chdir(current_path)
cfg = load_config(os.path.join(current_path, "demo/pose_cfg_multi.yaml"))

dataset = create_dataset(cfg)

sm = SpatialModel(cfg)
sm.load()

# Load and setup CNN part detector
sess, inputs, outputs = predict.setup_pose_prediction(cfg)


bridge = CvBridge()


def process_body_image(req):
    try:
        cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    image_batch = data_to_input(cv_image)
    outputs_np = sess.run(outputs, feed_dict={inputs: image_batch})
    scmap, locref, pairwise_diff = predict.extract_cnn_output(
        outputs_np, cfg, dataset.pairwise_stats)
    detections = extract_detections(cfg, scmap, locref, pairwise_diff)
    unLab, pos_array, unary_array, pwidx_array, pw_array = eval_graph(
        sm, detections)
    person_conf_multi = get_person_conf_multicut(
        sm, unLab, unary_array, pos_array)
    num_people = person_conf_multi.shape[0]
    conf_min_count = 3  # 至少三个点才被认为能够接受
    bodys_response = BodyPoseResponse()
    for pidx in range(num_people):
        if np.sum(person_conf_multi[pidx, :, 0] > 0) < conf_min_count:
            continue
        body_rect = person_conf_multi[pidx].tolist()
        body_rect_filterd = list(
            filter(lambda x: x[0] > 0 and x[1] > 0,  body_rect))
        if len(body_rect_filterd) < conf_min_count:
            continue
        body_info = BodyInfo()
        body_info.nose = body_rect[0]
        body_info.right_eye = body_rect[1]
        body_info.left_eye = body_rect[2]
        body_info.right_ear = body_rect[3]
        body_info.left_ear = body_rect[4]
        body_info.right_arm_top = body_rect[5]
        body_info.left_arm_top = body_rect[6]
        body_info.right_arm_middle = body_rect[7]
        body_info.left_arm_middle = body_rect[8]
        body_info.right_arm_bottom = body_rect[9]
        body_info.left_arm_bottom = body_rect[10]
        body_info.right_leg_top = body_rect[11]
        body_info.left_leg_top = body_rect[12]
        body_info.right_leg_middle = body_rect[13]
        body_info.left_leg_middle = body_rect[14]
        body_info.right_leg_bottom = body_rect[15]
        body_info.left_leg_bottom = body_rect[16]
        bodys_response.body_poses.append(body_info)
    return bodys_response


if __name__ == "__main__":
    rospy.init_node("body_pose_node", anonymous=True)
    rospy.Service('~get_body_pose', BodyPose, process_body_image)
    rospy.loginfo("body pose service started")
    rospy.spin()
