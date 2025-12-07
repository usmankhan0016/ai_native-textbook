---
sidebar_position: 4
sidebar_label: "Ch. 4: Synthetic Data Isaac Lab"
title: "Chapter 4: Synthetic Data Generation with Isaac Lab"
tags: ["isaac-lab", "synthetic-data", "domain-randomization", "machine-learning", "pytorch"]
difficulty: "Intermediate"
module: 3
week: 9
prerequisites: ["Chapter 1-3", "PyTorch basics", "Python 3.10+"]
estimated_time: "6-8 hours"
topics: ["Isaac Lab", "gymnasium environments", "domain randomization", "synthetic datasets", "transfer learning", "sim-to-real"]
---

# Chapter 4: Synthetic Data Generation with Isaac Lab

**Duration**: 6-8 hours | **Difficulty**: Intermediate | **Week**: 9

Generate unlimited training data automatically using domain randomization.

---

## Learning Objectives

By completing this chapter, you will be able to:

1. **Understand Why Synthetic Data Matters** — Explain cost/scale/labeling advantages over real-world data
2. **Set Up Isaac Lab Environment** — Create gymnasium-compatible RL environments for robotics
3. **Design Domain Randomization** — Randomize textures, lighting, objects, physics to improve sim-to-real
4. **Export Annotated Datasets** — Generate COCO-format datasets with automatic ground truth labels
5. **Integrate with PyTorch** — Load synthetic data into training pipelines
6. **Measure Domain Gap** — Evaluate transfer learning effectiveness (synthetic → real)
7. **Fine-Tune on Real Data** — Apply transfer learning to improve models on real-world data
8. **Scale Data Generation** — Automate generation of 100K+ frames per hour

---

## Key Concepts

### **Synthetic Data**
Training data generated entirely in simulation. Eliminates need for manual labeling, enables unlimited scale, reduces real-world robot testing. Critical for AI robotics due to cost of real data.

### **Domain Randomization**
Technique of randomizing visual/physical properties during training to prevent overfitting to simulation artifacts. Makes trained models robust to real-world variations.

### **Isaac Lab**
Gymnasium-compatible reinforcement learning framework for robotics. Includes built-in support for domain randomization, multi-robot simulation, and efficient data generation.

### **Transfer Learning**
Training a model on one dataset (synthetic), then fine-tuning on another (real). Typically 10-100x faster than training from scratch.

### **Sim-to-Real Gap**
Domain mismatch between simulation and reality. Objects look slightly different, physics isn't perfect, sensors have different noise. Domain randomization helps bridge this gap.

---

## Part 1: Why Synthetic Data?

### Cost Comparison

| Aspect | Real Data | Synthetic Data |
|--------|-----------|----------------|
| **Collection time** | 1000 images: 2-3 days | 1000 images: 5 minutes |
| **Labeling cost** | $10-20 per image | $0 (automatic) |
| **Total cost per 10K images** | $100K-200K | $100 (compute) |
| **Diversity** | Limited by environment | Unlimited (randomization) |
| **Scale** | Practical limit: 100K images | Achievable: 1M+ images |
| **Privacy** | Real objects/people | Fully synthetic |

**Bottom line**: Synthetic data is 100-1000x cheaper and enables 10x more training data.

### The Catch: Sim-to-Real Transfer

```
Model trained on synthetic data
    ↓ (usually performs worse on real data)
Accuracy on synthetic: 95%
Accuracy on real: 60%  ← Domain gap!
    ↓ (apply domain randomization during training)
Accuracy on synthetic: 85% (slightly lower due to randomization)
Accuracy on real: 92%  ← Much better transfer!
```

**Key insight**: Slight performance drop on synthetic → large gain on real.

---

## Part 2: Isaac Lab Environment Setup

### Creating a Domain Randomization Task

```python
# File: humanoid_task.py
"""
Isaac Lab task with domain randomization for humanoid perception.
"""

import torch
from omni.isaac.lab.assets import Articulation, RigidObject
from omni.isaac.lab.envs import ManagerBasedEnv, ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.utils import configclass

@configclass
class HumanoidPerceptionTaskCfg(ManagerBasedRLEnvCfg):
    """Configuration for humanoid perception task with domain randomization."""

    # Simulation settings
    sim: SimulationCfg = SimulationCfg(
        dt=0.01,
        render_interval=1,
        disable_contact_processing=False,
    )

    # Scene (robots, objects, lights)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=64,  # Parallel environments
        env_spacing=2.0,
        replicate_physics=True,
    )

    # Domain randomization parameters
    randomization = {
        # Lighting randomization
        "light_intensity_range": [0.5, 2.0],  # Vary brightness 0.5-2x
        "light_color_range": [(0.8, 0.8, 1.0), (1.0, 1.0, 0.8)],  # Warm/cool

        # Texture randomization
        "material_roughness_range": [0.1, 0.9],  # Glossy to rough
        "material_metallic_range": [0.0, 1.0],   # Non-metal to metal

        # Physics randomization
        "gravity_range": [8.0, 10.0],  # 8-10 m/s² (vs. 9.81)
        "friction_range": [0.3, 1.5],  # Low to high friction
        "restitution_range": [0.0, 0.3],  # Not bouncy to bouncy

        # Object randomization
        "object_scale_range": [0.5, 2.0],  # 0.5x-2x size
        "object_color_randomization": True,  # Random colors
        "num_obstacles_range": [0, 5],  # 0-5 obstacles in scene

        # Camera randomization
        "camera_noise_std": 0.01,  # Sensor noise
        "camera_distortion": True,  # Lens distortion
    }

class HumanoidPerceptionTask(ManagerBasedEnv):
    """Humanoid perception task for synthetic data generation."""

    cfg: HumanoidPerceptionTaskCfg

    def __init__(self, cfg: HumanoidPerceptionTaskCfg):
        super().__init__(cfg)

        # Get humanoid robot
        self.humanoid = self.scene.articulations["humanoid"]

        # Get objects in scene
        self.objects = self.scene.rigid_objects["objects"]

        # Observation space: RGB image + depth
        self.observation_spec = {
            "rgb": torch.Size([64, 640, 480, 3]),  # 64 envs, 640x480 RGB
            "depth": torch.Size([64, 640, 480]),   # Depth map
        }

    def _compute_observations(self):
        """Get observations from sensors."""
        obs = {}

        # Render RGB from camera
        obs["rgb"] = self.render_rgb()  # Shape: (64, 640, 480, 3)

        # Render depth from camera
        obs["depth"] = self.render_depth()  # Shape: (64, 640, 480)

        return obs

    def _compute_rewards(self):
        """Rewards (not used for data generation, just for RL training)."""
        return torch.zeros(self.cfg.scene.num_envs)

    def _compute_dones(self):
        """Check if episode is done."""
        return torch.zeros(self.cfg.scene.num_envs, dtype=torch.bool)

    def render_rgb(self):
        """Render RGB images from all parallel environments."""
        # Camera captures RGB for each of 64 parallel envs
        return torch.randn(64, 640, 480, 3) * 255  # Placeholder

    def render_depth(self):
        """Render depth maps from all parallel environments."""
        return torch.randn(64, 640, 480) * 10  # Placeholder

def make_env(cfg: HumanoidPerceptionTaskCfg = None):
    """Create Isaac Lab environment."""
    if cfg is None:
        cfg = HumanoidPerceptionTaskCfg()
    return HumanoidPerceptionTask(cfg)
```

### Running the Environment

```python
# File: generate_dataset.py
"""
Generate synthetic dataset using Isaac Lab with domain randomization.
"""

import torch
import numpy as np
import os
from PIL import Image
from humanoid_task import make_env, HumanoidPerceptionTaskCfg

# Create environment
cfg = HumanoidPerceptionTaskCfg()
env = make_env(cfg)

output_dir = "/tmp/synthetic_dataset_isaac_lab"
os.makedirs(output_dir, exist_ok=True)

# Generate 10,000 frames
for step in range(10000):
    # Step environment (applies domain randomization)
    obs, rewards, dones, info = env.step(torch.zeros(64, 10))  # 64 parallel envs

    # Extract observations
    rgb_images = obs["rgb"]  # Shape: (64, 640, 480, 3)
    depth_maps = obs["depth"]  # Shape: (64, 640, 480)

    # Save images
    for env_idx in range(64):
        frame_id = step * 64 + env_idx

        # RGB
        rgb = (rgb_images[env_idx].cpu().numpy() * 255).astype(np.uint8)
        rgb_path = f"{output_dir}/rgb_{frame_id:06d}.png"
        Image.fromarray(rgb).save(rgb_path)

        # Depth
        depth = depth_maps[env_idx].cpu().numpy().astype(np.uint16)
        depth_path = f"{output_dir}/depth_{frame_id:06d}.png"
        Image.fromarray(depth).save(depth_path)

    if step % 100 == 0:
        print(f"✅ Generated {step * 64} frames ({step / 100:.1f}% complete)")

print(f"✅ Dataset complete: {10000 * 64} frames in {output_dir}")
```

---

## Part 3: Domain Randomization in Detail

### Visual Randomization

```python
# Randomize every aspect of appearance
randomization_params = {
    # Lighting
    "sun_intensity": np.random.uniform(0.5, 2.0),  # Brightness
    "sun_azimuth": np.random.uniform(0, 360),      # Angle (degrees)
    "sun_elevation": np.random.uniform(10, 80),    # Height above horizon

    # Materials
    "object_color": np.random.uniform(0, 1, 3),    # RGB [0, 1]
    "material_roughness": np.random.uniform(0.1, 0.9),
    "material_metallic": np.random.uniform(0.0, 1.0),

    # Camera
    "focal_length": np.random.uniform(15, 35),     # mm (zoom variation)
    "aperture": np.random.uniform(1.4, 8.0),       # f-stop (depth of field)
}
```

### Physics Randomization

```python
# Randomize robot dynamics
physics_params = {
    "gravity": np.random.uniform(8.0, 10.0),       # Gravity strength
    "friction": np.random.uniform(0.3, 1.5),       # Friction coefficient
    "mass_scale": np.random.uniform(0.9, 1.1),     # ±10% mass variation
    "inertia_scale": np.random.uniform(0.9, 1.1),
    "damping": np.random.uniform(0.0, 0.5),
}
```

### Scene Randomization

```python
# Randomize environment
scene_params = {
    "num_obstacles": np.random.randint(0, 6),      # 0-5 obstacles
    "obstacle_size": np.random.uniform(0.1, 0.5),  # Size variation
    "obstacle_position": np.random.uniform(-2, 2, 3),  # Random placement
    "floor_texture": np.random.choice(["tile", "wood", "concrete"]),
    "background_color": np.random.uniform(0, 1, 3),  # Sky color
}
```

---

## Part 4: Transfer Learning

### Step 1: Train on Synthetic Data

```python
# File: train_synthetic.py
"""
Train object detection model on synthetic data from Isaac Lab.
"""

import torch
from torch.utils.data import DataLoader, Dataset
import torchvision.models as models

class SyntheticDataset(Dataset):
    def __init__(self, image_dir, label_file):
        self.images = [...]  # Load image paths
        self.labels = [...]  # Load COCO annotations

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        # Return image, bounding boxes, class labels
        return image, boxes, labels

# Create dataset
train_dataset = SyntheticDataset(
    image_dir="/tmp/synthetic_dataset_isaac_lab/rgb",
    label_file="/tmp/synthetic_dataset_isaac_lab/annotations.json"
)

# Train detection model on synthetic data
model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
optimizer = torch.optim.SGD(model.parameters(), lr=0.001)

for epoch in range(10):
    for batch in DataLoader(train_dataset, batch_size=32):
        images, targets = batch
        loss = model(images, targets)
        loss.backward()
        optimizer.step()

# Save synthetic-trained model
torch.save(model.state_dict(), "model_synthetic_trained.pt")

print("✅ Model trained on synthetic data")
```

### Step 2: Fine-Tune on Real Data

```python
# File: finetune_real.py
"""
Fine-tune model on real-world data.
"""

# Load synthetic-trained model
model = models.detection.fasterrcnn_resnet50_fpn(pretrained=False)
model.load_state_dict(torch.load("model_synthetic_trained.pt"))

# Load real-world dataset (usually much smaller)
real_dataset = RealDataset(
    image_dir="/path/to/real/images",
    label_file="/path/to/real/annotations.json"
)

# Fine-tune (lower learning rate to preserve synthetic knowledge)
optimizer = torch.optim.SGD(model.parameters(), lr=0.0001)

for epoch in range(5):  # Only 5 epochs (not 10 like synthetic)
    for batch in DataLoader(real_dataset, batch_size=8):
        images, targets = batch
        loss = model(images, targets)
        loss.backward()
        optimizer.step()

# Evaluate on test set
accuracy = evaluate(model, test_set)
print(f"✅ Fine-tuned model accuracy: {accuracy:.1%}")
```

### Transfer Learning Results

```
Training curve comparison:

Synthetic-only (no real data):
    Epoch 1: Accuracy 85% (on real test set) ← Sim-to-real gap!
    Epoch 5: Accuracy 88% (saturates, limited improvement)

Synthetic → Fine-tune on real:
    Start: Accuracy 85% (from synthetic training)
    Epoch 1: Accuracy 90% (rapid improvement from real data)
    Epoch 5: Accuracy 94% ← Much better!

Time comparison:
    Training from scratch: 48 hours (10K real images)
    Synthetic pre-training: 2 hours (100K synthetic images)
    Fine-tuning on real: 30 minutes (1K real images)
    Total: 2.5 hours (vs. 48 hours) ← 20x speedup!
```

---

## Part 5: Hands-On Labs

### Lab 1: Domain Randomization Experiment (2 hours)

Train two models:
1. **No randomization**: Same lighting, materials, physics every frame
2. **Full randomization**: Everything randomized per frame

Compare accuracy on real-world test set.

**Expected result**: Randomized model 10-15% higher real-world accuracy

### Lab 2: Scale Data Generation (1.5 hours)

Configure Isaac Lab with 64 parallel environments. Generate 50K frames in `<10` minutes.

**Success**: 50K frames exported, validated for ML training

### Lab 3: Transfer Learning Pipeline (2.5 hours)

Train on synthetic data → fine-tune on 1K real images. Compare to training from scratch.

**Success**: Fine-tuned model 20% faster to reach same accuracy

---

## Part 6: Code Examples

### Example: Sim-to-Real Gap Measurement

```python
# File: measure_domain_gap.py
"""
Quantify domain gap between synthetic and real data.
"""

def evaluate_model(model, synthetic_test_set, real_test_set):
    """Evaluate model on both synthetic and real data."""
    # Accuracy on synthetic
    synthetic_acc = evaluate(model, synthetic_test_set)

    # Accuracy on real
    real_acc = evaluate(model, real_test_set)

    # Domain gap
    gap = synthetic_acc - real_acc

    print(f"Synthetic accuracy: {synthetic_acc:.1%}")
    print(f"Real accuracy:      {real_acc:.1%}")
    print(f"Domain gap:         {gap:.1%}")

    return gap

# Models without randomization
model_no_random = load_model("model_no_randomization.pt")
gap_no_random = evaluate_model(model_no_random, synthetic_test, real_test)
# Output: Domain gap: 35% (bad!)

# Models with randomization
model_with_random = load_model("model_with_randomization.pt")
gap_with_random = evaluate_model(model_with_random, synthetic_test, real_test)
# Output: Domain gap: 8% (much better!)
```

---

## Part 7: End-of-Chapter Exercises

### Exercise 1: Generate Large-Scale Dataset (1.5 hours)

Use Isaac Lab to generate 100K frames with domain randomization.

**Acceptance**: 100K frames exported, file size ~40-60 GB

### Exercise 2: Domain Randomization Ablation (2 hours)

Train three models:
1. No randomization
2. Light randomization only
3. Full randomization

Measure real-world accuracy for each.

**Acceptance**: Report shows full randomization significantly better

### Exercise 3: Transfer Learning Speedup (1.5 hours)

Compare:
1. Training from scratch on 10K real images: X hours
2. Synthetic pre-training + fine-tune on 1K real: Y hours

Measure speedup: X / Y

**Acceptance**: 10-50x speedup, `<10%` accuracy loss

### Exercise 4: Synthetic-Real Parity (2 hours)

Train model on synthetic until it matches real-world performance.

Document:
- Synthetic accuracy
- Real accuracy
- Number of frames needed
- Domain gap

**Acceptance**: Gap `<10%`, documented training curves

---

## Part 8: Capstone Integration

Synthetic data (Chapter 4) trains your Humanoid AI Assistant's perception:

- **Chapter 2**: Humanoid digital twin + sensors
- **Chapter 3**: Real-time perception pipeline (inference)
- **Chapter 4**: Generate training data at scale ← YOU ARE HERE
- **Chapter 5**: Deploy optimized model to Jetson

Your capstone requires:
- ✅ 10K+ synthetic training images
- ✅ Detection model trained to >80% accuracy
- ✅ Fine-tuned on small real dataset (if available)
- ✅ Transfer learning documented

---

## Next Steps

In Chapter 5, you'll deploy your trained model to real hardware (Jetson) with safety mechanisms.

Ready? Move to **[Chapter 5: End-to-End Deployment](./chapter-5-deployment.md)**.

---

**Chapter Summary**: 6-8 hours | Difficulty: Intermediate | Week 9-10

---

## Additional Resources

- [Isaac Lab Documentation](https://isaac-lab.readthedocs.io/)
- [Domain Randomization Paper](https://arxiv.org/abs/1703.06907)
- [COCO Dataset Format](https://cocodataset.org/)
- [PyTorch Transfer Learning](https://pytorch.org/tutorials/beginner/transfer_learning_tutorial.html)
