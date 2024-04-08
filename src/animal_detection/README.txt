As of 31st Mar 2024:

Dataset Augmentation:
Created dir animal_detection inside src. Placed backup animals data in it.
1. Took test, train folders from backup data (animals folder) and upload entire thing to Roboflow. (Using gatech email)
2. In Roboflow, performed split (Train:70, Val: 20, Test: 10)
3. Used Roboflow to apply Training dataset Augmentation. Now I have the dataset.
4. Exported dataset in yolov3-keras format.

Training
Followed [1] and its accompanying colab notebook [2].
1. Created anaconda env pa_detection_env.
2. Created training_script.ipynb, copied from train.py
3. Made changes to a dependency file (encode/decode stuff)
4. Modified generate_random_data function to include

**************************************

7th Apr 2024
Agenda: Training YoloV5, so the dir keras-yolov3 can be blown away if this works. 
1. Export the dataset created on 31st in yolov5 pytorch format from roboflow.
2. Create dir yolov5 inside animal_detection.
3. Cloned yolov5 repo [4]
4. Created anaconda env called pa_yolov5_env with python 3.9. Ran pip install requirements.txt using requirements file from yolov5.
5. Following [3]. Created custom_yolov5.yaml, only changed nc=31.
6. Changed dir structure for it all to work.
7. cd yolov5 and run the command for training: python train.py --img 416 --batch 16 --epochs 100 --data ./data/data.yaml --cfg ./models/custom_yolov5s.yaml --weights '' --name yolov5s_results  --cache


**************************************
References
1. https://blog.roboflow.com/training-a-yolov3-object-detection-model-with-a-custom-dataset/#how-to-train-a-yolov3-model-for-object-detection  
2. https://colab.research.google.com/drive/1ByRi9d6_Yzu0nrEKArmLMLuMaZjYfygO?ref=blog.roboflow.com#scrollTo=4hBFndz8VeI6
3. https://blog.roboflow.com/how-to-train-yolov5-on-a-custom-dataset/ 
4. https://github.com/ultralytics/yolov5 