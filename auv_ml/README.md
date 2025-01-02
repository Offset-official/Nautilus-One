## Usage

Ensure that you use `rosdep` for installing all required packages. If `numpy` gives you an error, install any `numpy==1.*.*` using `pip`.

Run server:
```bash
ros2 run auv_ml yolo_inference_server
```

Run client:
```bash
ros2 run auv_ml yolo_inference_test <input_img> <output_img>
```


## Wiki
1. `yolo_inference_test` is just for testing right now. The service will be used by the camera later on. 
2. I have defined all dependencies in `package.xml` using `rosdep`. If you still get errors, contact Subham.

