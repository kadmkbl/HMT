import tensorflow as tf
import random
import numpy as np

def representative_data_gen():
  for _ in range(100):
    data = np.random.rand(1, 224, 224, 3)
    yield [data.astype(np.float32)]

converter = tf.compat.v1.lite.TFLiteConverter.from_keras_model_file('./Origin_Food_New.h5')
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen

# Ensure that if any ops can't be quantized, the converter throws an error
# converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]

# Set the input and output tensors to uint8 (APIs added in r2.3)
converter.inference_input_type = tf.uint8
converter.inference_output_type = tf.uint8

tflite_model = converter.convert()

interpreter = tf.lite.Interpreter(model_content=tflite_model)
input_type = interpreter.get_input_details()[0]['dtype']
print('input: ', input_type)
output_type = interpreter.get_output_details()[0]['dtype']
print('output: ', output_type)

# Save the model.
with open('Origin_Food_New_Quantized_all.tflite', 'wb') as f:
  f.write(tflite_model)
