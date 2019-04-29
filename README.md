####"人体关节点识别"
[body_pose](https://github.com/BluewhaleRobot/body_pose) 是一个人体姿态识别的软件包。这个软件包可以从图片中识别出人体的耳朵，眼睛，鼻子，四肢的共17个特征点。其实现是通过深度学习网络利用tensorflow框架。此软件还支持多人同时识别。

(https://community.bwbot.org/assets/uploads/files/1530878219879-6f71b239-27ae-453a-94d9-94de429c8c84-image-resized.png) 

### 安装
1. 安装tensorflow
由于小强上面没有Nvidia显卡,所以我们安装CPU版本的Tensorflow。小强的CPU支持一些高效的指令集，默认的Tensorflow为了能够在更多平台上运行，没有使用这些指令集。性能实际上没有发挥到最高。我们可以安装开启了这些优化指令的版本。

```bash
sudo pip2 install --ignore-installed --upgrade "https://github.com/lakshayg/tensorflow-build/releases/download/tf1.12.0-macOS-mojave-ubuntu16.04-py2-py3/tensorflow-1.12.0-cp27-cp27mu-linux_x86_64.whl"
```

2. 安装软件包相关依赖

```bash
sudo pip install scipy scikit-image matplotlib pyyaml easydict cython munkres==1.0.12
```

3. 下载相关模型

```
git clone https://github.com/BluewhaleRobot/body_pose
# 单人模型
cd src/models/mpii
./download_models.sh
$ cd -
# 多人模型

./compile.sh
cd models/coco
./download_models.sh
cd -
```

### 运行

1. 运行单人识别测试程序

```bash
# 在src文件夹内运行
TF_CUDNN_USE_AUTOTUNE=0 python demo/singleperson.py
```

运行成功后可以看到下图

![0_1530879542859_94996fd8-2ca8-418f-ac9f-11da6221380a-image.png](https://community.bwbot.org/assets/uploads/files/1530879564082-94996fd8-2ca8-418f-ac9f-11da6221380a-image-resized.png) 

2. 运行多人识别的例子

```bash
TF_CUDNN_USE_AUTOTUNE=0 python demo/demo_multiperson.py
```

成功运行后显示下面的图像

![0_1530879857466_158a04a4-defc-498a-87e3-fdae8ee9ab41-image.png](https://community.bwbot.org/assets/uploads/files/1530879880168-158a04a4-defc-498a-87e3-fdae8ee9ab41-image-resized.png) 

3. 运行ROS服务

  ①把该功能包下载到catkin_ws/src目录下：
  
  ②而修改body_pose_test.launch内容为：
  <node name="body_pose" pkg="body_pose" type="body_pose_node.py" output="screen"></node>
   <node name="body_pose_tester" pkg="body_pose" type="body_pose_test_node.py" output="screen"></node>
   <include file="$(find usb_cam)/launch/usb_cam-test.launch"/>
   
  ③然后修改"body_pose_test_node.py的~get_body_pose为/body_pose/get_body_pose;  ~image为/usb_cam/image_raw
  
  ④然后rosrun image_view image_view image:=/body_pose_tester/processed_image
  
*（插入）python画图函数：
  #画一条厚度为5像素的蓝色斜线
  img = cv2.line(img,(x1,y1),(x2,y2),"颜色 (b,g,r)",width)

  #绘制矩形，需要矩形的左上角和右下角。这次我们将在图像的右上角绘制一个绿色矩形。
  img = cv2.rectangle(img,(x1,y1),(x2,y2),"颜色 (255,0,0)",width)

  #绘制圆，需要其中心坐标和半径。我们将在上面绘制的矩形内绘制一个圆。
  img = cv2.circle(img,(x1,y1), r , "颜色 (b,g,r)", thickness=-1)
  thickness 如果是正数，表示组成圆的线条的粗细程度。否则，表示圆是否被填充

  #绘制椭圆,要绘制椭圆，我们需要传递几个参数。一个参数是中心位置（x，y）。
  #下一个参数是轴长度（长轴长度，短轴长度）。角度是椭圆在逆时针方向上的旋转角度。
  img = cv2.ellipse(img,(256,256),(100,50),0,0,180,255,-1)


  
