

import os
import numpy as np
from PIL import Image

import tensorflow.compat.v1.keras as keras
from keras.models import load_model
from keras.preprocessing import image
from keras_applications.resnext import ResNeXt50, preprocess_input
from keras.applications.imagenet_utils import decode_predictions
# import tensorflow as tf
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

# disable most logs
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# ROS-independent class for the detector
class EngagementDetector:
    def __init__(self, test=False):
        self._this_dir_path = os.path.dirname(os.path.realpath(__file__))

        self.resNet = None
        self.resNet_graph = None
        self.lstm = None
        self.lstm_graph = None
        self.window_size = 0

        # tf settings
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess= tf.Session(config=config)
        keras.backend.set_session(sess)

        # load the networks in memory
        self._load_networks(test=test)


    def _load_networks(self, test=False):
        # load resnet
        self.resNet = ResNeXt50(include_top=test, input_shape=(224, 224, 3),  pooling='max', weights='imagenet', backend = keras.backend, layers = keras.layers, models = keras.models, utils = keras.utils)
        self.resNet._make_predict_function()
        self.resNet_graph = tf.get_default_graph()

        ##### test resnet in detecting a dog in img
        ##### NOTE: set `include_top=True` above to test this
        if test:
            img_path = os.path.join(self._this_dir_path, "../imgs/dog.png")
            img = image.load_img(img_path, target_size=(224, 224))
            img = image.img_to_array(img)
            x = preprocess_input(np.expand_dims(img.copy(), axis=0), backend = keras.backend, layers = keras.layers, models = keras.models, utils = keras.utils)
            preds = self.resNet.predict(x)
            print("\n\n\n>> TEST: dog prediction\n", decode_predictions(preds, top=5))
            print("\n\n\n")
            
        # load lstm
        model_path = os.path.join(self._this_dir_path, "/models/lstm_10_50_runsigm_runsigm.h5")
        self.lstm = load_model(model_path)
        self.lstm._make_predict_function()
        self.lstm_graph = tf.get_default_graph()

        # this model is trained to detect engagement on 10 frames
        self.window_size = 10   # frames

    # frames_seq is a sequence of 10 frames 
    def predict(self, frames_seq):
        if len(frames_seq) != self.window_size:
            return None

        batch = []

        frames_seq = [np.array(Image.fromarray(frame).resize((224, 224))) for frame in frames_seq]
        batch = preprocess_input(np.array(frames_seq), backend = keras.backend, layers = keras.layers, models = keras.models, utils = keras.utils)

        # extract features with resnet
        with self.resNet_graph.as_default():
            features = self.resNet.predict_on_batch(batch)

        # predict engagement with lstm
        with self.lstm_graph.as_default():
            prediction = self.lstm.predict(features[np.newaxis,:])

        return prediction

if __name__ == "__main__":
    det = EngagementDetector(test=True)