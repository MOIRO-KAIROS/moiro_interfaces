# faceRec_ros2 Using Adaface and YOLOv8
#### This repo is designed to implement real-time face recognition in ROS2
##### [ Ubuntu 22.04 ROS2 humble ver. ]
## 0. YOLOv8_ros
https://github.com/mgonzs13/yolov8_ros
- Use for Person detection
  
## 0. Adaface
https://github.com/mk-minchul/AdaFace
- Use for Face Recognition

## 1. Setup
### 1) conda 설치
#### 참고: https://webnautes.tistory.com/1844 # 레퍼런스

-  Architecture 확인
    ```
    uname -a # x86_64

    sudo apt-get update && sudo apt-get upgrade
    ```
- nvidia driver 설치

    ```
    apt --installed list | grep nvidia-driver # 설치할 수 있는 드라이버 버전을 확인
    sudo apt-get install nvidia-driver-525 # sudo apt install는 옛날버전이므로 X
    sudo reboot
    ```
- 잘 설치했는지 확인
    ```
    nvidia-smi
    sudo apt-get update && sudo apt-get upgrade
    ```
- cuda 설치 (11.8 ver)
    ```
    wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
    sudo sh cuda_11.8.0_520.61.05_linux.run
    ```
    > 참고 페이지(https://webnautes.tistory.com/1844)에서 터미널 설정 확인: Continue / accept / Driver 해제 / Install

- 환경변수 추가
    ```
    vim ~/.bashrc
    ```
    > export PATH="/usr/local/cuda-11.8/bin:$PATH"
    
    > export LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH"

    ```
    source ~/.bashrc
    ```
- CUDA 잘 설치했는지 확인
    ```
    nvcc --version
    ```

### 2) 가상환경 설치 (pytorch 설치)
```
conda create --name adaface python=3.9 && conda activate adaface
conda activate adaface
pip install pyyaml
pip install typeguard
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install scikit-image matplotlib pandas scikit-learn
pip install pytorch pytorch-lightning==1.8.6
pip install tqdm bcolz-zipline prettytable menpo mxnet opencv-python
pip install -U colcon-common-extensions
```


```
conda list | grep torch
```
> pytorch-lightning         1.8.6                    pypi_0    pypi

> torch                     2.2.2+cu118              pypi_0    pypi

> torchaudio                2.2.2+cu118              pypi_0    pypi

> torchmetrics              1.3.2                    pypi_0    pypi

> torchvision               0.17.2+cu118             pypi_0    pypi


### 3) Installation
- (주의) workspace 이름을 moiro_ws로 준수할 것
```
  cd ~/moiro_ws/src
  git clone
  pip3 install -r faceRec_ros2/requirements.txt
  cd ~/moiro_ws
  colcon build
```

### 4) RUN *** 전에 할 것
#### (1)  ```pretrained``` 폴더 생성 후, weight(.ckpt) 다운로드
- 다운로드 (링크 클릭)
    | Arch | Dataset    | Link                                                                                         |
    |------|------------|----------------------------------------------------------------------------------------------|
    | R50  | MS1MV2     | [gdrive](https://drive.google.com/file/d/1eUaSHG4pGlIZK7hBkqjyp2fc2epKoBvI/view?usp=sharing) |

- 파일 구조 (추가해야하는 pretrained와 video 부분 폴더만 깊게 표기)
    ```
    faceRec_ros2
    ├── adaface
    │   ├── adaface
    │   │   ├── adaface_ros2.py
    │   │   ├── __init__.py
    │   │   └── script
    │   │       ├── adaface.py
    │   │       ├── embed
    │   │       ├── face_alignment
    │   │       ├── face_dataset
    │   │       ├── __init__.py
    │   │       ├── LICENSE
    │   │       ├── main.py
    │   │       ├── net.py
    │   │       ├── pretrained
    │   │       ├── __pycache__
    │   │       ├── README.md
    │   │       ├── requirements.txt
    │   │       ├── scripts
    │   │       └── utils.py
    │   ├── launch
    │   ├── package.xml
    │   ├── resource
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── test   
    │    
    ├── README.md
    └── yolov8_ros

    ```

### 5) Usage
```
ros2 launch adaface adaface.launch.py
```

![image](https://github.com/MOIRO-KAIROS/faceRec_ros2/assets/114575723/d955b345-c5c8-4efe-b6c2-4ccd349e8470)
![rosgraph](https://github.com/MOIRO-KAIROS/faceRec_ros2/assets/114575723/f34f394e-b0a0-4348-86ac-77c6ff51baf1)


## 하단부 수정 필요
### 4) Run Inference
```
python inference.py
```

> warning이 뜬다면? 그냥 진행해도 OK, 아니면 오류문구 보고 고치기 (모르겠으면 바로 질문!!)

### 5) Run Demo File (WebCam)


1. ```python3 0_store_embedding.py``` # face_dataset/test에 있는 얼굴들에 대한 특징값 추출 후 저장
2. ```python3 1_run_recognition.py``` # webcam 활성화 후, demo file 실행

### 6) Run Demo File (.mp4)

1. video/iAm.zip 압축풀기 > iAm.mp4
    - 파일 구조
    ```
    video
      |
      |_____ iAm.mp4
    ```
2. ```python3 0_store_embedding.py```
3. ```python3 2_test_recognition.py``` # mp4에 대한 face Recognition 수행
    ```
    video_capture = cv2.VideoCapture('video/iAM.mp4') # 경로 설정 후 실행하기
    ```
