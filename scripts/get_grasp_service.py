#! /usr/bin/env python

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
import rospy
from mmp_grasp_detect.srv import GetGrasp
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
import cv2

class GraspService:
    def __init__(self):
        self.model = self.build_model()
        self.model.compile(
            loss=keras.losses.MeanSquaredError(),
            optimizer=keras.optimizers.RMSprop(learning_rate=0.01),
            metrics=['mae']
        )
        self.model.load_weights('/home/ollie/mmp_ws/src/mmp_grasp_detect/config/weights2.h5')
        self.bridge = CvBridge()

    @staticmethod
    def build_model():
        inputs = keras.Input(shape=(400, 400, 1))
        x = layers.Conv2D(32, 3)(inputs)
        x = layers.BatchNormalization()(x)
        x = keras.activations.relu(x)
        x = layers.MaxPooling2D()(x)
        x = layers.Conv2D(64, 3)(x)
        x = layers.BatchNormalization()(x)
        x = keras.activations.relu(x)
        x = layers.MaxPooling2D()(x)
        x = layers.Conv2D(128, 5)(x)
        x = layers.BatchNormalization()(x)
        x = keras.activations.relu(x)
        x = layers.Flatten()(x)
        x = layers.Dropout(0.35)(x)
        x = layers.Dense(512, activation='relu')(x)
        x = layers.Dense(256, activation='relu')(x)
        outputs = layers.Dense(6)(x)
        model = keras.Model(inputs=inputs, outputs=outputs)
        return model


    def get_grasp_cb(self, req):
        try:
            image = self.bridge.imgmsg_to_cv2(req.image, "passthrough")
        except CvBridgeError as e:
            print(e)

        image_tensor = tf.convert_to_tensor(image, dtype=tf.float32)
        image_gray = tf.image.rgb_to_grayscale(image_tensor)
        img = tf.expand_dims(image_gray, axis=0)
        img = tf.image.per_image_standardization(img)

        # cv2.imshow("Image to detect grasp from.", image)
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        prediction = self.model.predict(img) / 100
        return prediction.tolist()


    def get_grasp_server(self):
        rospy.init_node('grasp_getter_service')
        srv = rospy.Service('get_grasp', GetGrasp, self.get_grasp_cb)
        print("Ready to find grasps!")
        rospy.spin()

if __name__ == "__main__":
    gs = GraspService()
    gs.get_grasp_server()