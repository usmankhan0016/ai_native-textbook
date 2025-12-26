# Chapter 2: Vision for VLA – Building Perception Pipelines

**Module**: 4 | **Week**: 11 | **Difficulty**: Intermediate | **Estimated Time**: 8–10 hours

---

## Learning Objectives

After completing this chapter, you will be able to:

1. **Design and implement** multi-modal perception pipelines combining RGB, depth, and segmentation data
2. **Apply state-of-the-art models** (YOLO, Mask R-CNN, SAM, Grounding DINO) for object detection and affordance extraction
3. **Integrate ROS 2 perception nodes** that subscribe to camera and depth topics and publish structured scene information
4. **Extract affordances** from visual observations to inform LLM reasoning (e.g., "which objects are graspable?")
5. **Fuse multiple sensor modalities** (RGB + depth + segmentation) into a scene graph suitable for LLM planning

---

## Key Concepts

- **Object Detection** (YOLO): Fast, real-time identification of object classes and bounding boxes
- **Instance Segmentation** (Mask R-CNN): Per-object pixel-level masks enabling precise graspability analysis
- **Foundation Models** (SAM): Segment Anything Model for zero-shot segmentation of any object
- **Grounding Vision-Language Models** (Grounding DINO): Link natural language descriptions to visual regions
- **Affordance Detection**: Identifying how objects can be interacted with (graspable, movable, pushable, etc.)
- **Scene Graphs**: Structured representations of objects, their properties, and spatial relationships
- **RGB-D Fusion**: Combining color information with depth for 3D scene understanding

---

## Part 1: Perception Fundamentals

### Perception Pipeline for VLA

A VLA robot must answer: "What is in the environment, where is it, and how can I interact with it?"

The perception pipeline transforms raw sensor data into LLM-readable information:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Sensor Input Layer                                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                          │
│  │RGB Camera│  │D-Cam/ToF │  │ (Optional)                          │
│  │1920x1080 │  │640x480   │  │ LiDAR    │                          │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                          │
└──────┼─────────────┼──────────────┼──────────────────────────────────┘
       │             │              │
       ↓             ↓              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                    Model Inference Layer                             │
│  ┌──────────────────┐      ┌──────────────────┐                    │
│  │ YOLO Detection   │      │ Mask R-CNN       │                    │
│  │ (Fast detection) │      │ (Instance masks) │                    │
│  └────────┬─────────┘      └────────┬─────────┘                    │
└──────────┼──────────────────────────┼──────────────────────────────┘
           ↓                          ↓
┌─────────────────────────────────────────────────────────────────────┐
│                    Feature Extraction Layer                          │
│  ┌──────────────────┐      ┌──────────────────┐                    │
│  │ Depth Inference  │      │ Affordance Models│                    │
│  │ (3D coords)      │      │ (Graspability)   │                    │
│  └────────┬─────────┘      └────────┬─────────┘                    │
└──────────┼──────────────────────────┼──────────────────────────────┘
           ↓                          ↓
┌─────────────────────────────────────────────────────────────────────┐
│                    Scene Graph Construction                          │
│  {                                                                   │
│    "objects": [                                                      │
│      {"id": "cup_0", "class": "cup", "color": "red",                │
│       "location": [0.5, 0.3, 0.8], "graspable": true},              │
│      {"id": "table_0", "class": "table", "graspable": false}        │
│    ],                                                                │
│    "spatial_relationships": [                                        │
│      {"subject": "cup_0", "predicate": "on", "object": "table_0"}   │
│    ]                                                                 │
│  }                                                                   │
└─────────────────────────────────────────────────────────────────────┘
           ↓
┌─────────────────────────────────────────────────────────────────────┐
│                  LLM-Ready Scene Representation                      │
│  "There is a red cup on the table at position (0.5m, 0.3m, 0.8m).   │
│   The cup is graspable. The table is not interactive."              │
└─────────────────────────────────────────────────────────────────────┘
```

### Camera & Depth Sensors

**RGB Cameras**: Standard color images (1920×1080 or higher). Provides semantic information (color, text, fine details).

**Depth Sensors**: Measure distance from camera to objects. Common types:
- **Stereo Vision**: Uses two RGB cameras + computation to infer depth (e.g., Intel RealSense D435)
- **Time-of-Flight (ToF)**: Directly measures light travel time (e.g., Kinect, PMD CamBoard)
- **LiDAR**: Laser-based scanning (e.g., Livox Mid360, SICK LMS1xx) – coarse but long-range

For household VLA robots, **RGB-D pairs** (RGB + depth from same viewpoint) are standard.

### Object Detection Models

**YOLO (You Only Look Once)**: Fast, real-time object detection.
- Input: RGB image
- Output: Bounding boxes + class labels + confidence scores
- Speed: ~30-60 FPS on GPU (fast enough for real-time control)
- Accuracy: ~80-90% on common objects (COCO dataset)
- Use when: You need fast detection of common objects

**Mask R-CNN**: Slower but provides instance masks (pixel-level object boundaries).
- Input: RGB image
- Output: Bounding boxes + class labels + per-pixel segmentation masks
- Speed: ~5-10 FPS (slower, but more precise)
- Accuracy: ~90%+ with high precision
- Use when: You need to know exact object shapes for grasping

**SAM (Segment Anything Model)**: Zero-shot segmentation of any object.
- Input: RGB image + optional prompt (point or bounding box)
- Output: Precise mask for any object (even ones not in training data)
- Speed: ~50-100ms per mask (acceptable for planning, not real-time control)
- Accuracy: Exceptional on novel objects
- Use when: You encounter unknown objects not in training data

**Grounding DINO**: Link natural language to visual regions.
- Input: RGB image + text description
- Output: Bounding box + mask for the described region
- Speed: ~200ms (good for planning, not control)
- Accuracy: Excellent at finding described objects
- Use when: LLM outputs "find the red cup" and you need the visual region

### Object Affordance Detection

Affordances describe **how objects can be interacted with**. A robot needs to know:
- Is this object **graspable**? (Can I pick it up?)
- Is it **movable**? (Can I push it?)
- Is it **fragile**? (Should I use gentle forces?)
- Is it **openable**? (Does it have lids, doors, drawers?)

Affordance detection can be:
1. **Learned from data**: Train a classifier on affordance labels (e.g., graspable vs. non-graspable)
2. **Rule-based**: Use heuristics (e.g., "small objects are graspable, tables are not")
3. **Semantic**: Use LLM reasoning ("glasses are fragile, so handle gently")

For the capstone, we'll use a **simple learned classifier** trained on affordance labels.

### Scene Graphs

A **scene graph** represents the environment as a structured data format:

```json
{
  "timestamp": "2025-12-07T10:30:00Z",
  "frame_id": "camera_link",
  "objects": [
    {
      "id": "obj_001",
      "class": "cup",
      "color": "red",
      "confidence": 0.92,
      "bbox": [450, 300, 550, 400],  // [x_min, y_min, x_max, y_max] in pixels
      "mask": null,  // Optionally include pixel-level mask
      "position_3d": [0.45, 0.30, 0.80],  // [x, y, z] in meters (camera frame)
      "affordances": {
        "graspable": true,
        "fragile": false,
        "movable": true
      }
    },
    {
      "id": "obj_002",
      "class": "table",
      "confidence": 0.98,
      "position_3d": [0.50, 0.0, 0.00],
      "affordances": {
        "graspable": false,
        "climbable": false,
        "pushable": false
      }
    }
  ],
  "relationships": [
    {
      "subject": "obj_001",
      "predicate": "on",
      "object": "obj_002"
    }
  ]
}
```

The scene graph can then be converted to natural language for the LLM:
> "There is a red cup (graspable) at position (0.45m, 0.30m, 0.80m) on a table at (0.50m, 0.0m, 0.00m)."

---

## Part 2: Multi-Modal Fusion

### RGB + Depth Fusion

Combining RGB and depth creates a powerful representation:
- **RGB provides**: Semantic understanding (what objects are), fine details (colors, text, logos)
- **Depth provides**: 3D locations, object shapes, surface geometry

**Fusion Challenge**: Alignment. RGB and depth images have different resolutions and may be captured from slightly different viewpoints.

**Solution**: Project depth into RGB frame using camera intrinsics:

```python
# File: perception/rgb_d_fusion.py
import numpy as np
import cv2

def project_depth_to_rgb(depth_image, camera_intrinsics):
    """
    Project depth image to RGB frame using camera intrinsics.

    Args:
        depth_image: Depth map (640x480) in meters
        camera_intrinsics: Camera K matrix (3x3)

    Returns:
        RGB frame with depth overlay
    """
    h, w = depth_image.shape
    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]

    # Create point cloud from depth
    x = np.arange(w)
    y = np.arange(h)
    xx, yy = np.meshgrid(x, y)

    z = depth_image.astype(np.float32) / 1000.0  # Convert mm to m
    x_3d = (xx - cx) * z / fx
    y_3d = (yy - cy) * z / fy
    z_3d = z

    # Filter points (remove zeros, too far)
    valid = (z > 0) & (z < 5.0)  # 0-5 meters

    return {
        'x': x_3d[valid],
        'y': y_3d[valid],
        'z': z_3d[valid],
        'valid_mask': valid
    }
```

**Output**: For each pixel in the RGB image, we now know:
- 2D position (row, column)
- 3D position (x, y, z in camera frame)
- Color (R, G, B from RGB image)

This enables precise object localization for grasping.

### Multi-View Camera Fusion

Many robots use multiple cameras for full-body perception (front, side, top). Fusing views requires:

1. **Timestamp Synchronization**: Ensure images are captured at nearly the same time
2. **Frame Alignment**: Transform objects detected in each camera's frame into a common robot frame
3. **De-duplication**: Recognize when two detections refer to the same object in the world

**ROS 2 Approach**: Use the **tf2 library** for frame transformations:

```python
# File: perception/multi_view_fusion.py
import tf2_ros
from geometry_msgs.msg import TransformStamped

class MultiViewFusion:
    def __init__(self, node):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.objects = {}  # Global object registry

    def fuse_detection(self, detection, camera_frame):
        """
        Fuse a detection from one camera into the global object map.

        Args:
            detection: Detection (class, bbox, position_3d)
            camera_frame: Frame ID (e.g., "front_camera_link")
        """
        # Transform detection from camera frame to robot base frame
        transform = self.tf_buffer.lookup_transform(
            "base_link", camera_frame, rospy.Time(0)
        )

        # Apply transformation to 3D position
        position_3d = self._transform_point(
            detection.position_3d, transform
        )

        # Check if we've seen this object before
        object_id = self._find_matching_object(
            detection.class_name, position_3d
        )

        if object_id:
            # Update existing object
            self.objects[object_id].update(detection, position_3d)
        else:
            # Create new object
            object_id = f"obj_{len(self.objects)}"
            self.objects[object_id] = DetectedObject(
                class_name=detection.class_name,
                position_3d=position_3d,
                confidence=detection.confidence
            )

        return object_id

    def _find_matching_object(self, class_name, position_3d, distance_threshold=0.2):
        """Check if we've seen this object before (within distance threshold)."""
        for obj_id, obj in self.objects.items():
            if obj.class_name == class_name:
                dist = np.linalg.norm(np.array(obj.position_3d) - np.array(position_3d))
                if dist < distance_threshold:
                    return obj_id
        return None
```

### Affordance Detection

A simple affordance classifier learns to predict how objects can be interacted with:

```python
# File: perception/affordance_classifier.py
import torch
from torchvision import models, transforms

class AffordanceClassifier:
    def __init__(self, model_path):
        # Load pretrained ResNet-50 backbone
        self.model = models.resnet50(pretrained=True)
        # Replace final layer for affordance classification
        num_affordances = 6  # graspable, movable, fragile, openable, pushable, stackable
        self.model.fc = torch.nn.Linear(2048, num_affordances)
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                std=[0.229, 0.224, 0.225])
        ])

    def predict(self, image, detection_bbox):
        """
        Predict affordances for an object detected in the image.

        Args:
            image: RGB image (numpy array)
            detection_bbox: [x_min, y_min, x_max, y_max] in pixels

        Returns:
            affordances: {'graspable': 0.95, 'movable': 0.87, ...}
        """
        # Crop object region from image
        x_min, y_min, x_max, y_max = detection_bbox
        object_crop = image[y_min:y_max, x_min:x_max]

        # Preprocess
        input_tensor = self.transform(object_crop).unsqueeze(0)

        # Inference
        with torch.no_grad():
            logits = self.model(input_tensor)
            probabilities = torch.sigmoid(logits)[0]  # Multi-label (object can be both movable and fragile)

        affordance_names = ['graspable', 'movable', 'fragile', 'openable', 'pushable', 'stackable']
        return {name: float(prob) for name, prob in zip(affordance_names, probabilities)}
```

---

## Part 3: ROS 2 Perception Integration

### ROS 2 Perception Node Architecture

A perception node in ROS 2 follows this pattern:

```python
# File: ros2_perception_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np

class PerceptionNode(Node):
    """
    ROS 2 perception node that:
    1. Subscribes to RGB and depth topics
    2. Runs detection and segmentation models
    3. Publishes scene graph (as a JSON message or custom message type)
    """

    def __init__(self):
        super().__init__('perception_node')

        # Subscribers
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10
        )
        self.depth_subscription = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )

        # Publishers
        self.scene_graph_publisher = self.create_publisher(String, '/perception/scene_graph', 10)
        self.detection_image_publisher = self.create_publisher(Image, '/perception/detections_viz', 10)

        # Initialize detector (YOLO)
        from ultralytics import YOLO
        self.detector = YOLO('yolov8m.pt')  # Medium model: good balance

        # Bridge for ROS2 <-> OpenCV conversion
        self.bridge = CvBridge()

        # Storage for latest frames
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_intrinsics = None

    def rgb_callback(self, msg):
        """Process RGB image."""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def depth_callback(self, msg):
        """Process depth image."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def camera_info_callback(self, msg):
        """Store camera intrinsics."""
        self.camera_intrinsics = np.array(msg.K).reshape(3, 3)

    def process_frame(self):
        """Main processing: detect, extract affordances, build scene graph."""
        if self.latest_rgb is None or self.latest_depth is None:
            return

        # Run YOLO detection
        results = self.detector(self.latest_rgb)
        detections = results[0]

        scene_graph = {
            'timestamp': self.get_clock().now().to_msg(),
            'objects': []
        }

        for det in detections.boxes:
            # Extract bounding box and class
            x_min, y_min, x_max, y_max = map(int, det.xyxy[0])
            class_id = int(det.cls[0])
            confidence = float(det.conf[0])
            class_name = self.detector.names[class_id]

            # Get 3D position from depth
            depth_crop = self.latest_depth[y_min:y_max, x_min:x_max]
            if depth_crop.size > 0:
                center_depth = np.nanmedian(depth_crop)
                if np.isfinite(center_depth):
                    cx = (x_min + x_max) // 2
                    cy = (y_min + y_max) // 2
                    x_3d = (cx - self.camera_intrinsics[0, 2]) * center_depth / self.camera_intrinsics[0, 0]
                    y_3d = (cy - self.camera_intrinsics[1, 2]) * center_depth / self.camera_intrinsics[1, 1]
                    z_3d = center_depth

                    scene_graph['objects'].append({
                        'id': f'{class_name}_{len(scene_graph["objects"])}',
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': [x_min, y_min, x_max, y_max],
                        'position_3d': [float(x_3d), float(y_3d), float(z_3d)],
                        'affordances': {
                            'graspable': class_name not in ['table', 'floor', 'wall'],
                            'movable': class_name in ['cup', 'bottle', 'book', 'toy']
                        }
                    })

        # Publish scene graph
        from std_msgs.msg import String
        import json
        msg = String()
        msg.data = json.dumps(scene_graph)
        self.scene_graph_publisher.publish(msg)

        # Publish visualization
        viz_image = self.latest_rgb.copy()
        for obj in scene_graph['objects']:
            x_min, y_min, x_max, y_max = obj['bbox']
            cv2.rectangle(viz_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(viz_image, f"{obj['class']} ({obj['confidence']:.2f})",
                       (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
        self.detection_image_publisher.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Points**:
- Subscribes to RGB and depth topics (standard ROS 2 message types)
- Runs YOLO detection on RGB
- Extracts 3D coordinates using depth + camera intrinsics
- Publishes scene graph as JSON (or custom message type)
- Includes visualization for debugging

---

## Part 4: Labs & Exercises

### Lab 1: Detect Objects in a Scene Image (1 hour)

**Objective**: Use YOLO to detect objects in an image and evaluate accuracy.

**Materials Needed**:
- Python environment with YOLO and OpenCV installed
- Sample images (can use COCO validation set or your own photos)
- ~1 hour

**Instructions**:

1. **Install dependencies**:
   ```bash
   pip install ultralytics opencv-python numpy
   ```

2. **Load and run YOLO**:
   ```python
   from ultralytics import YOLO
   import cv2

   # Load model
   model = YOLO('yolov8m.pt')

   # Run inference
   image_path = 'sample_image.jpg'
   results = model(image_path)

   # Analyze results
   for detection in results[0].boxes:
       print(f"Class: {detection.cls}, Confidence: {detection.conf}")
   ```

3. **Evaluate accuracy**:
   - Run on 5-10 images
   - Count true positives (correct detections)
   - Count false positives (incorrectly detected)
   - Count false negatives (missed objects)
   - Calculate: Precision = TP / (TP + FP), Recall = TP / (TP + FN)

4. **Visualize results**:
   - Draw bounding boxes on images
   - Save visualization

**Acceptance Criteria**:
- [ ] YOLO successfully detects objects in sample images
- [ ] At least 3 detections analyzed with confidence scores
- [ ] Precision and recall calculated from 5+ images
- [ ] Visualization saved showing bounding boxes

**Estimated Time**: 1 hour

---

### Lab 2: Identify Pickable Objects (1.5 hours)

**Objective**: Build a classifier to identify which objects are graspable/pickable.

**Materials Needed**:
- Image dataset with graspable/non-graspable labels (or create your own)
- PyTorch and torchvision
- ~1.5 hours

**Instructions**:

1. **Prepare dataset**:
   - Collect or download object images (e.g., COCO or Google Images)
   - Label each as graspable (cup, ball, toy) or non-graspable (table, wall, floor)
   - Split into train (70%) and test (30%)

2. **Train a classifier**:
   ```python
   import torch
   from torchvision import models, transforms
   from torch.utils.data import DataLoader, TensorDataset

   # Load pretrained ResNet
   model = models.resnet18(pretrained=True)
   model.fc = torch.nn.Linear(512, 2)  # Binary classification

   # Training loop
   optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
   loss_fn = torch.nn.CrossEntropyLoss()

   for epoch in range(10):
       for images, labels in train_loader:
           logits = model(images)
           loss = loss_fn(logits, labels)
           loss.backward()
           optimizer.step()
   ```

3. **Evaluate on test set**:
   - Run on test images
   - Calculate accuracy, precision, recall

4. **Visualize predictions**:
   - Show images with predicted graspability

**Acceptance Criteria**:
- [ ] Dataset created with 50+ labeled images
- [ ] Model trained with training and validation loss plotted
- [ ] Test accuracy ≥ 75%
- [ ] Confusion matrix or per-class metrics reported

**Estimated Time**: 1.5 hours

---

### Lab 3: Convert Scene to LLM-Readable Format (1 hour)

**Objective**: Transform detected objects into a natural language scene description suitable for an LLM.

**Materials Needed**:
- Detected objects (from Lab 1) with bounding boxes and positions
- LLM API (mock or real, optional)
- ~1 hour

**Instructions**:

1. **Convert detections to structured data**:
   ```python
   detections = [
       {'class': 'cup', 'color': 'red', 'position': [0.5, 0.3, 0.8], 'graspable': True},
       {'class': 'table', 'position': [0.5, 0.0, 0.0], 'graspable': False},
   ]
   ```

2. **Extract spatial relationships**:
   - For each pair of objects, determine relationship (on, next_to, behind, in_front_of)
   - Example: cup is `on` the table

3. **Generate natural language description**:
   ```python
   def scene_to_text(detections):
       text = "Scene description:\n"
       for det in detections:
           text += f"- A {det['color']} {det['class']} at position {det['position']}\n"
       # Add relationships
       text += "\nRelationships:\n"
       text += "- The red cup is on the table\n"
       return text
   ```

4. **Prompt an LLM** (optional):
   ```python
   from openai import OpenAI

   client = OpenAI()
   prompt = f"""
   {scene_text}

   User command: "Pick up the red cup"

   Task plan (as JSON):
   """

   response = client.chat.completions.create(
       model="gpt-4",
       messages=[{"role": "user", "content": prompt}]
   )
   ```

**Acceptance Criteria**:
- [ ] Detections converted to structured data format
- [ ] Spatial relationships extracted from at least 3 pairs of objects
- [ ] Natural language scene description generated
- [ ] (Optional) LLM prompted with scene and returned task decomposition

**Estimated Time**: 1 hour

---

### End-of-Chapter Exercises

**Exercise 1: Model Comparison** (Beginner)
- Compare YOLO, Mask R-CNN, and SAM on a sample image
- Create a table comparing speed, accuracy, and use cases

**Exercise 2: Affordance Labeling** (Intermediate)
- Given a set of objects, design affordance labels
- Create a rule-based classifier (if-then rules for graspability)
- Test on household objects

**Exercise 3: Scene Graph Construction** (Advanced)
- Design a complete scene graph format for VLA
- Include: objects, affordances, spatial relationships
- Implement JSON schema with validation

---

## Capstone Integration

Chapter 2 teaches you to **perceive and understand** the environment. In the capstone:
1. You'll run a real-time perception pipeline on a simulated scene
2. The scene graph you build will feed into the LLM (Chapter 3)
3. The LLM will reason about which objects are graspable, movable, etc.
4. ROS 2 controllers (Chapter 4) will execute the resulting task plan

---

## Summary

Vision is the first pillar of VLA. Robots must perceive their environment to reason about tasks. Key takeaways:

- **Object detection** (YOLO) provides fast, real-time identification
- **Depth fusion** enables 3D localization for grasping
- **Affordance detection** teaches robots how to interact with objects
- **Scene graphs** provide structured, LLM-ready representations
- **ROS 2 integration** makes perception part of the robot's middleware stack

---

## References

1. Redmon, J., & Farhadi, A. (2018). "YOLOv3: An Incremental Improvement." arXiv:1804.02767
2. He, K., et al. (2017). "Mask R-CNN." ICCV
3. Kirillov, A., et al. (2023). "Segment Anything." arXiv:2304.02135
4. Shi, Z., et al. (2024). "Grounding DINO: Marrying DINO with Grounded Pre-Training." arXiv:2401.14207

---

**Next Chapter**: [Chapter 3: Language Planning with Whisper & LLMs](./chapter-3-language-planning-whisper-llm.md)
