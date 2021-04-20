"""
This python script contains the definition and training functions of my first tensorflow CNN prototype.
This was done using all inbuilt loss and training functions. This created a model that would learn the grasps
up to a certain point, but not as accurate as hoped (hence building a custom model).
Author: olt13
"""

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from TensorflowPrototype.load_dataset import LoadData
import pandas as pd

physical_devices = tf.config.list_physical_devices('GPU')
for gpu in physical_devices:
    tf.config.experimental.set_memory_growth(gpu, True)


class TFProto:
    def __init__(self):
        self.load_data = False
        print("Loading data.....")
        self.image_loc = '/home/ollie/Documents/Major project/acronym data/dataset_images2/'
        if self.load_data:
            self.load_data = LoadData()
            self.grasp_df = self.load_data.load()
        else:
            self.grasp_df = pd.read_csv('/home/ollie/Documents/Major project/acronym data/dataset.csv')
        self.ds_train = None
        self.load_model = False
        try:
            if self.load_model:
                self.model = keras.models.load_model('/home/ollie/Documents/Major project/acronym data/tensorflow_model')
                print("Model Loaded")
            else:
                raise OSError
        except OSError:
            self.model = self.build_model()
            self.model.compile(
                loss=keras.losses.MeanAbsoluteError(),
                optimizer=keras.optimizers.Adam(learning_rate=1),
                metrics=['mse', 'mae', 'mape']
            )
        print(self.model.summary())

    # Build keras CNN model
    @staticmethod
    def build_model():
        inputs = keras.Input(shape=(400, 400, 3))
        x = layers.Conv2D(32, 1)(inputs)
        x = layers.BatchNormalization()(x)
        x = keras.activations.relu(x)
        x = layers.MaxPooling2D()(x)
        x = layers.Conv2D(64, 1)(x)
        x = layers.BatchNormalization()(x)
        x = keras.activations.relu(x)
        x = layers.MaxPooling2D()(x)
        x = layers.Conv2D(128, 3)(x)
        x = layers.BatchNormalization()(x)
        x = keras.activations.relu(x)
        x = layers.MaxPooling2D()(x)
        x = layers.Conv2D(256, 3)(x)
        x = layers.BatchNormalization()(x)
        x = keras.activations.relu(x)
        x = layers.Flatten()(x)
        x = layers.Dense(128, activation='relu')(x)
        x = layers.Dense(64, activation='relu')(x)
        x = layers.Dense(32, activation='relu')(x)
        outputs = layers.Dense(6, activation='linear')(x)
        model = keras.Model(inputs=inputs, outputs=outputs)
        return model

    def process_data(self):
        target_cols = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        file_paths = self.grasp_df['image'].values
        labels = self.grasp_df[target_cols].values
        self.ds_train = tf.data.Dataset.from_tensor_slices((file_paths, labels))
        self.ds_train = self.ds_train.map(self.read_image).batch(8)

    def read_image(self, image_file, label):
        image = tf.io.read_file(self.image_loc + image_file)
        image = tf.image.decode_image(image, dtype=tf.float32)
        return image, label

    def show_data(self):
        print(self.grasp_df)

    def train(self):
        self.model.fit(self.ds_train, batch_size=32, epochs=1, verbose=1)

    def eval(self):
        # self.model.evaluate(self.x_test, self.y_test, batch_size=64, verbose=2)
        pass

    def save(self):
        self.model.save('/home/ollie/Documents/Major project/acronym data/tensorflow_model')


def main():
    prototype = TFProto()
    prototype.process_data()
    prototype.train()
    prototype.save()
    # prototype.eval()


if __name__ == "__main__":
    main()
