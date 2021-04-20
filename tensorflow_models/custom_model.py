"""
This python script contains the definition and training functions of my custom built tensorflow CNN.
I have built in order to see if the custom loss function makes a difference to the final outcome,
i.e. the final grasp success in the simulation, and improves the predicted grasps accuracy.
Author: olt13
"""

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import pandas as pd
import matplotlib.pyplot as plt
import progressbar

physical_devices = tf.config.list_physical_devices('GPU')
for gpu in physical_devices:
    tf.config.experimental.set_memory_growth(gpu, True)


def process_data():
    target_cols = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    grasp_df = pd.read_csv('dataset.csv')     #.sample(frac=1).reset_index(drop=True)
    images = grasp_df['image'].values
    labels = grasp_df[target_cols].values
    x_train = images
    y_train = labels
    ds_train = tf.data.Dataset.from_tensor_slices((x_train, y_train))
    ds_train = ds_train.map(read_image).batch(10)

    iterator = iter(ds_train)
    # print(iterator.get_next()[1][0].numpy())
    # print(self.grasp_df[target_cols].iloc[0].to_numpy())
    # Check that the tensors are correctly transformed
    assert iterator.get_next()[1][0].numpy().all() == grasp_df[target_cols].iloc[0].to_numpy().all()
    return ds_train


def read_image(image_file, label):
    image = tf.io.read_file('/home/ollie/Documents/Major project/scw1780/images/' + image_file)
    image = tf.image.decode_image(image, dtype=tf.float32)
    image = tf.image.rgb_to_grayscale(image)
    image = tf.image.per_image_standardization(image)
    label = tf.cast(label, tf.float32)
    return image, label


def grasp_loss(loss_obj, labels, y_pred):
    losses = []
    for y in labels[:1]:
        loss = y - y_pred
        losses.append(sum(loss))

    closest = min(losses)
    return loss_obj(closest, y_pred)


def build_model():
    inputs = keras.Input(shape=(400, 400, 1))
    x = layers.Conv2D(32, 3)(inputs)
    x = layers.BatchNormalization()(x)
    x = keras.activations.relu(x)
    x = layers.MaxPooling2D()(x)
    x = layers.Conv2D(64, 3)(x)
    x = layers.BatchNormalization()(x)
    x = keras.activations.relu(x)
    # x = layers.MaxPooling2D()(x)
    # x = layers.Conv2D(128, 5)(x)
    # x = layers.BatchNormalization()(x)
    # x = keras.activations.relu(x)
    x = layers.Flatten()(x)
    x = layers.Dropout(0.35)(x)
    # x = layers.Dense(128, activation='relu')(x)
    x = layers.Dense(16, activation='relu')(x)
    outputs = layers.Dense(6)(x)
    model = keras.Model(inputs=inputs, outputs=outputs)

    return model


def training_loop(model, dataset):
    optimizer = keras.optimizers.RMSprop(learning_rate=0.001)
    loss_obj = keras.losses.MeanAbsoluteError()
    train_loss_results = []

    epochs = 10 + 1
    widgets = [progressbar.SimpleProgress(), '  ', progressbar.Bar(), '  ', progressbar.ETA()]
    b = progressbar.ProgressBar(len(dataset))
    for epoch in range(1, epochs):
        epoch_loss_avg = tf.keras.metrics.Mean()

        print("Epoch: {e}/{n}".format(e=epoch, n=epochs-1))
        b.start()
        for index, (x, y) in enumerate(dataset):
            with tf.GradientTape() as tape:
                inp = tf.expand_dims(x[0], axis=0)
                loss = grasp_loss(loss_obj, y, model(x))
                # print(loss)
            grads = tape.gradient(loss, model.trainable_variables)

            optimizer.apply_gradients(zip(grads, model.trainable_variables))
            epoch_loss_avg.update_state(loss)
            b.update(index)

        print("Epoch loss: {l}".format(l=epoch_loss_avg.result()))
        train_loss_results.append(epoch_loss_avg.result())
        b.finish()

    model.save_weights("custom_model_weights.h5")
    plt.plot(train_loss_results)
    plt.xlabel('Epoch')
    plt.ylabel('MAE to closest grasp')
    plt.show()


def main():
    dataset = process_data()
    model = build_model()
    training_loop(model, dataset)


if __name__ == '__main__':
    main()
