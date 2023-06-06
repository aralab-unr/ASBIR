# ASBIR-Deep-Learning
Deep learning models

The basemodels prior to manipulation came from UNR ARA lab and Chuong Le. The models and image processing have been manipulated to work for the rust dataset and ENet was added into the models to train. 

To start the vitual enviroment:

source tensorflow/venv/bin/activate


To train the model:

python3 -m keras_segmentation train --checkpoints_path="results/annette/checkpoints/" --train_images="dataset/rust_img/" --train_annotations="dataset/mask_rust/" --val_images="dataset/rust_img/" --val_annotations="dataset/mask_rust/" --n_classes=2 --input_height=512 --input_width=512 --model_name="enet" --epoch=400 --batch_size=4



To test the model:

python3 -m keras_segmentation predict_test --checkpoints_path="results/annette/checkpoints/checkpoint-389-0.9872-.hdf5" --input_path="dataset/rust_img/" --output_path="results/annette/imageOutput/"
