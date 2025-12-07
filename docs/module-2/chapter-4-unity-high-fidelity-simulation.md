---
sidebar_position: 4
sidebar_label: "Ch.4: Unity High-Fidelity"
title: "Chapter 4: High-Fidelity Simulation in Unity"
tags: ["unity", "rendering", "photorealistic", "simulation", "week-7"]
difficulty: "Advanced"
module: 2
week: 7
prerequisites: ["Chapters 1-3", "Unity 2022 LTS"]
estimated_time: "4-5 hours"
topics: ["unity-robotics", "ros2-communication", "rendering-pipelines", "dataset-export", "domain-randomization"]
---

# Chapter 4: High-Fidelity Simulation in Unity

**Estimated time**: 4-5 hours | **Difficulty**: Advanced | **Week**: 7, Days 4-5

## Learning Objectives

After completing this chapter, you will be able to:

1. **Explain why Unity is valuable** for high-fidelity robotics visualization and training data generation
2. **Compare PhysX with Gazebo physics engines** and understand their respective strengths
3. **Set up a Unity robotics project** with required packages and ROS 2 bridge
4. **Establish ROS 2 ↔ Unity communication** using the ros2cs library
5. **Import humanoid models** into Unity with correct materials and physics
6. **Simulate grasping and object interaction** using D6 joint constraints
7. **Configure photorealistic rendering** using HDRP (High-Definition Render Pipeline)
8. **Export synthetic training datasets** with ground truth annotations

## Key Concepts

1. **PhysX**: NVIDIA's physics engine used by Unity (different from Gazebo's ODE/Bullet/DART)
2. **ros2cs**: C# bindings that allow Unity scripts to communicate with ROS 2
3. **HDRP vs URP**: High-Definition (photorealistic, GPU-intensive) vs Universal (mobile, fast) rendering pipelines
4. **Domain randomization**: Vary lighting, materials, camera pose to create robust training datasets
5. **Ground truth**: Perfect annotations (pose, depth, segmentation) generated in simulation

---

## Why Unity for Robotics?

### Advantages

✅ **High-fidelity rendering**: Photorealistic visuals with HDRP pipeline
✅ **Real-time interactivity**: Optimized for interactive simulations and demos
✅ **Training data**: Generate large synthetic datasets with perfect annotations
✅ **Visual debugging**: Easy to see humanoid movement and interactions
✅ **VR/AR integration**: Support for virtual/augmented reality environments
✅ **Large ecosystem**: Thousands of assets, tools, and community resources

### Disadvantages

❌ **Physics accuracy**: PhysX is optimized for games, not research-grade physics
❌ **Long-horizon simulations**: Not ideal for simulating hours of behavior
❌ **Batch simulations**: Harder to run hundreds of parallel simulations
❌ **ROS 2 integration**: Requires bridge library (ros2cs)

### Recommended Use

Use **Unity for**:
- Visualization of humanoid behavior
- Training dataset generation
- Real-time interactive control
- Stakeholder demos

Use **Gazebo for**:
- Physics validation and algorithm testing
- Long-duration simulations
- Batch running multiple scenarios
- Non-real-time accurate physics

---

## PhysX vs. Gazebo Physics Engines

### Comparison Table

| Aspect | PhysX (Unity) | ODE (Gazebo) | Bullet (Gazebo) | DART (Gazebo) |
|--------|---|---|---|---|
| **Speed** | Fast | Medium | Faster | Slower |
| **Accuracy** | Medium | High | Medium | Very High |
| **Stability** | Good | Good | Variable | Excellent |
| **Humanoid Dynamics** | Moderate | Good | Moderate | Excellent |
| **Ease of Use** | Easy (visual editor) | Moderate (SDF tuning) | Easy | Hard (many parameters) |
| **Best For** | Real-time, visualization | Research, validation | Prototyping | Humanoid research |

### When to Use Each

**PhysX (Unity)**:
- Real-time visualization needed
- Interactive control and feedback
- Training data with lighting variety
- Rapid prototyping

**ODE (Gazebo)**:
- Physics algorithm validation
- Accurate long-horizon simulation
- Publication-quality research
- Joint control loops

**Bullet (Gazebo)**:
- Speed-critical non-real-time sims
- Fast iteration during development

**DART (Gazebo)**:
- Humanoid-specific dynamics
- Contact-rich interactions
- Maximum physics fidelity

---

## Setting Up Unity for Robotics

### Step 1: Install Unity 2022 LTS

Download from [Unity Hub](https://unity.com/download):
1. Install Unity Hub
2. Add Unity 2022 LTS version
3. Create new 3D project

### Step 2: Install ros2cs Plugin

The ros2cs library provides C# bindings for ROS 2:

```bash
# In your Unity project root (outside Assets folder):
git clone https://github.com/ROS2ForUnity/ROS2-Unity.git
# Copy ROS2 folder into Assets/
```

### Step 3: Configure Project Settings

In Unity Editor:
- **Window → TextMesh Pro → Import TMP Essentials**
- **Edit → Project Settings → Player → Scripting → .NET Framework 4.7.1 (or later)**
- **Assets → ROS2 → ROS2 Settings**: Configure ROS 2 domain ID and middleware

---

## ROS 2 ↔ Unity Communication via ros2cs

### Publishing to ROS 2 (Send Joint States)

```csharp
// File: Assets/Scripts/JointStatePublisher.cs
using UnityEngine;
using ROS2;
using geometry_msgs.msg;
using sensor_msgs.msg;

public class JointStatePublisher : MonoBehaviour
{
    private ROS2Node ros2Node;
    private IPublisher<JointState> jointPub;
    private Animator humanoidAnimator;

    void Start()
    {
        // Initialize ROS 2
        ros2Node = GetComponent<ROS2Node>();
        jointPub = ros2Node.CreatePublisher<JointState>("joint_states");
        humanoidAnimator = GetComponent<Animator>();
    }

    void Update()
    {
        // Publish joint states at 30 Hz
        var msg = new JointState();
        msg.Header.Stamp = ROS2.Clock.Now;
        msg.Name = new string[] { "left_hip_x", "left_hip_y", "left_knee" };
        msg.Position = new double[]
        {
            humanoidAnimator.GetFloat("LeftHipX"),
            humanoidAnimator.GetFloat("LeftHipY"),
            humanoidAnimator.GetFloat("LeftKnee")
        };
        jointPub.Publish(msg);
    }
}
```

### Subscribing from ROS 2 (Receive Commands)

```csharp
// File: Assets/Scripts/JointCommandSubscriber.cs
using UnityEngine;
using ROS2;
using sensor_msgs.msg;

public class JointCommandSubscriber : MonoBehaviour
{
    private ROS2Node ros2Node;
    private ISubscription<JointState> cmdSub;
    private Animator humanoidAnimator;

    void Start()
    {
        ros2Node = GetComponent<ROS2Node>();
        cmdSub = ros2Node.CreateSubscription<JointState>(
            "cmd_joints",
            JointCommandCallback
        );
        humanoidAnimator = GetComponent<Animator>();
    }

    void JointCommandCallback(JointState msg)
    {
        // Update animator parameters from ROS 2 command
        for (int i = 0; i < msg.Name.Length; i++)
        {
            humanoidAnimator.SetFloat(msg.Name[i], (float)msg.Position[i]);
        }
    }
}
```

---

## Importing Humanoid Models

### From URDF to Unity

1. **Export URDF as URDF.zip** (includes meshes)
2. **Use URDF importer**:
   ```bash
   # In Assets folder:
   Assets/ROS2/URDF Importer → Import URDF
   ```
3. **Configure PhysX**:
   - Each body link becomes GameObject with Rigidbody
   - Colliders auto-added from collision geometry
   - Joints converted to ConfigurableJoint

### Configure Materials and Lighting

```csharp
// File: Assets/Scripts/MaterialConfigurator.cs
using UnityEngine;

public class MaterialConfigurator : MonoBehaviour
{
    void Start()
    {
        // Find humanoid mesh renderer
        Renderer meshRenderer = GetComponent<Renderer>();

        // Create PBR material with HDRP shader
        Material material = new Material(Shader.Find("HDRP/Lit"));
        material.SetColor("_BaseColor", new Color(0.5f, 0.5f, 0.5f, 1.0f));
        material.SetFloat("_Metallic", 0.5f);     // 0=non-metal, 1=full metal
        material.SetFloat("_Smoothness", 0.7f);  // 0=rough, 1=glossy

        meshRenderer.material = material;
    }
}
```

---

## Grasping and Manipulation

### D6 Joint Constraint (Grasping)

```csharp
// File: Assets/Scripts/GraspController.cs
using UnityEngine;

public class GraspController : MonoBehaviour
{
    private ConfigurableJoint graspJoint;
    private bool isGrasping = false;

    void Start()
    {
        // Hand GameObject
        GameObject hand = transform.Find("right_hand").gameObject;

        // Add D6 joint for grasping
        graspJoint = hand.AddComponent<ConfigurableJoint>();
        graspJoint.xMotion = ConfigurableJointMotion.Free;
        graspJoint.yMotion = ConfigurableJointMotion.Free;
        graspJoint.zMotion = ConfigurableJointMotion.Free;
    }

    void OnTriggerEnter(Collider col)
    {
        // Grasp object when hand touches it
        if (col.CompareTag("Graspable"))
        {
            graspJoint.connectedBody = col.GetComponent<Rigidbody>();
            isGrasping = true;
        }
    }

    void OnTriggerExit(Collider col)
    {
        // Release when no longer in contact
        if (isGrasping && col.CompareTag("Graspable"))
        {
            graspJoint.connectedBody = null;
            isGrasping = false;
        }
    }
}
```

---

## High-Fidelity Rendering

### HDRP Pipeline Setup

1. **Create HDRP Project** or **upgrade existing project**:
   - Package Manager → Search "High Definition RP"
   - Install latest version
   - Create HDRP asset and set as default

### HDRP Material Configuration

```csharp
// File: Assets/Scripts/HDRPMaterialSetup.cs
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class HDRPMaterialSetup : MonoBehaviour
{
    void SetupHDRPMaterial(Renderer rend, float metallic, float smoothness)
    {
        Material mat = rend.material;

        // Base color
        mat.SetColor("_BaseColor", new Color(0.7f, 0.7f, 0.7f, 1.0f));

        // Metallic: 0=non-metal, 1=metal
        mat.SetFloat("_Metallic", metallic);

        // Smoothness: 0=rough, 1=glossy
        mat.SetFloat("_Smoothness", smoothness);

        // Normal map (if available)
        // mat.SetTexture("_NormalMap", normalTexture);

        // Ambient occlusion (for realism)
        // mat.SetTexture("_AmbientOcclusionMap", aoTexture);
    }

    void Start()
    {
        // Configure humanoid materials
        SetupHDRPMaterial(GetComponent<Renderer>(), 0.3f, 0.6f);  // Rubber-like

        // Configure floor
        Transform floor = transform.Find("Floor");
        SetupHDRPMaterial(floor.GetComponent<Renderer>(), 0.1f, 0.3f);  // Concrete
    }
}
```

### Lighting for Photorealism

```csharp
// File: Assets/Scripts/LightingSetup.cs
using UnityEngine;

public class LightingSetup : MonoBehaviour
{
    void Start()
    {
        // Directional light (sun)
        GameObject sunGO = new GameObject("Sun");
        Light sunLight = sunGO.AddComponent<Light>();
        sunLight.type = LightType.Directional;
        sunLight.intensity = 2.0f;
        sunLight.colorTemperature = 6500f;  // Daylight
        sunGO.transform.rotation = Quaternion.Euler(45, 0, 0);

        // Point light (task light)
        GameObject lampGO = new GameObject("TaskLight");
        Light lamp = lampGO.AddComponent<Light>();
        lamp.type = LightType.Point;
        lamp.intensity = 1.5f;
        lamp.range = 5.0f;
        lampGO.transform.position = new Vector3(0, 2, 0);

        // Environment: skybox for reflections
        // (Set in Lighting settings → Skybox)
    }
}
```

---

## Exporting Training Datasets

### Ground Truth Annotation Format

```json
{
  "frame_id": 0,
  "timestamp": 1234567890.123,
  "humanoid_pose": {
    "position": [0.5, 0.0, 1.0],
    "rotation_quaternion": [0, 0, 0, 1]
  },
  "camera": {
    "intrinsics": {
      "fx": 320.0,
      "fy": 320.0,
      "cx": 320.0,
      "cy": 240.0
    },
    "pose": {
      "position": [0, 0, 0],
      "rotation": [0, 0, 0, 1]
    }
  },
  "objects": [
    {
      "id": 1,
      "name": "ball",
      "pose": [1.0, 0, 0.5, 0, 0, 0, 1],
      "bounding_box": [[100, 100], [200, 200]],
      "segmentation_id": 42
    }
  ]
}
```

### Dataset Exporter Script

```csharp
// File: Assets/Scripts/DatasetExporter.cs
using UnityEngine;
using System.IO;
using System.Collections.Generic;

public class DatasetExporter : MonoBehaviour
{
    public Camera captureCamera;
    public int frameCount = 100;
    public string outputPath = "Assets/GeneratedDataset/";

    void Start()
    {
        if (!Directory.Exists(outputPath))
            Directory.CreateDirectory(outputPath);

        StartCoroutine(ExportFrames());
    }

    System.Collections.IEnumerator ExportFrames()
    {
        for (int i = 0; i < frameCount; i++)
        {
            // Capture RGB image
            Texture2D screenshot = ScreenCapture.CaptureScreenshotAsTexture();
            byte[] imageData = screenshot.EncodeToPNG();
            File.WriteAllBytes($"{outputPath}/frame_{i:04d}.png", imageData);

            // Generate ground truth annotation
            Dictionary<string, object> annotation = new Dictionary<string, object>();
            annotation["frame_id"] = i;
            annotation["timestamp"] = Time.realtimeSinceStartup;

            // Humanoid pose (would need to read from animator)
            string jsonString = JsonUtility.ToJson(annotation);
            File.WriteAllText($"{outputPath}/frame_{i:04d}_annotation.json", jsonString);

            yield return new WaitForEndOfFrame();
        }

        Debug.Log($"Exported {frameCount} frames to {outputPath}");
    }
}
```

---

## Domain Randomization

Vary visual appearance to create robust datasets:

```csharp
// File: Assets/Scripts/DomainRandomizer.cs
using UnityEngine;

public class DomainRandomizer : MonoBehaviour
{
    public void RandomizeScene()
    {
        // Randomize lighting
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            light.intensity = Random.Range(0.5f, 3.0f);
            light.colorTemperature = Random.Range(3000f, 8000f);
        }

        // Randomize material colors
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        foreach (Renderer rend in renderers)
        {
            Material mat = rend.material;
            mat.SetColor("_BaseColor", Random.ColorHSV());
        }

        // Randomize camera position
        Camera cam = GetComponent<Camera>();
        float offset = Random.Range(-0.2f, 0.2f);
        cam.transform.Translate(offset, offset, 0);
    }
}
```

---

## Hands-On Labs

### Lab 1: Set Up Unity ROS 2 Bridge

**Objective**: Create ROS 2 ↔ Unity communication and verify data flow.

**Steps**:
1. Create empty humanoid in Unity
2. Add ROS2Node component
3. Attach JointStatePublisher script
4. Animate humanoid, monitor ROS 2 topic:
   ```bash
   ros2 topic echo /joint_states
   ```

**Verification**: Joint states appear in terminal

### Lab 2: Import Humanoid and Test Grasping

**Objective**: Import humanoid URDF, add grasping constraint, pick up object.

**Steps**:
1. Import humanoid.urdf into Unity
2. Add GraspController script to hand
3. Create graspable object (cube with Graspable tag)
4. Run simulation, move hand to touch object
5. Hand should grasp and move object

### Lab 3: Photorealistic Rendering and Dataset Export

**Objective**: Set up HDRP, render high-quality scene, export 100-frame dataset.

**Steps**:
1. Switch to HDRP pipeline
2. Configure materials and lighting
3. Attach DatasetExporter with DomainRandomizer
4. Run and export frames:
   ```bash
   ls GeneratedDataset/frame_*.png | wc -l
   # Should show 100 files
   ```

---

## End-of-Chapter Exercises

### Exercise 1: ROS 2 Communication Pipeline

**Task**: Implement 2-way communication: send sine-wave joint commands from ROS 2 node, observe humanoid movement in Unity.

### Exercise 2: Grasping Evaluation

**Task**: Simulate picking up 5 different objects (varying mass 1-10kg), measure grasp success rate.

### Exercise 3: Rendering Quality Comparison

**Task**: Compare all 3 rendering pipelines (Built-in, URP, HDRP) on same scene. Document frame rates and visual quality.

### Exercise 4: Dataset Generation and Validation

**Task**: Export 1000-frame dataset with domain randomization. Validate JSON annotations are syntactically correct.

---

## Summary

✅ **Why Unity**: Real-time rendering, training data, visualization

✅ **PhysX vs Gazebo**: Trade-offs between accuracy and speed

✅ **ros2cs communication**: C# ROS 2 integration for control

✅ **Humanoid import**: URDF to Unity with physics

✅ **Grasping**: D6 joint constraints for manipulation

✅ **Photorealistic rendering**: HDRP materials and lighting

✅ **Dataset export**: Ground truth annotations for ML training

✅ **Domain randomization**: Visual variety for robust models

---

## Capstone Preparation

You've now completed:
- ✅ Chapter 1: Digital Twin concepts
- ✅ Chapter 2: Physics simulation (Gazebo)
- ✅ Chapter 3: Sensor simulation
- ✅ Chapter 4: High-fidelity rendering (Unity)

**Week 8 Capstone**: Combine all skills to simulate a humanoid completing a real task (walk, navigate, grasp, etc.) in both Gazebo and Unity.

You're ready for [Week 6 Practice Guide](./week-6.md) and [Week 7 Practice Guide](./week-7.md)!
