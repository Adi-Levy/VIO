# Monocular VO/VIO Project High-Level Design (HLD)

## 1. Goal
Build a pure C++ monocular visual-inertial odometry library with a thin ROS 2 wrapper. The system should work with cheap hardware first (single camera + hobby IMU), remain usable without ROS, and be extensible to better sensors and alternative visual processing modes later.

## 2. Current design decisions

### Locked decisions
- Core implementation language: C++
- Middleware separation: pure C++ core + thin ROS 2 wrapper
- Initial visual processing mode: Shi-Tomasi + KLT
- Planned later visual processing mode: ORB
- Estimator family for VIO: filter-style VIO
- Low-level vision/geometry library: OpenCV is acceptable for optimized primitives
- Initial execution path: offline runner before ROS integration

### Naming / terminology
To avoid confusion with web-development terminology:
- **Visual processing** = feature detection, tracking, matching, filtering
- **Motion geometry** = essential matrix, pose recovery, triangulation, PnP later
- **State estimator** = VO/VIO state tracking, propagation, correction, covariance

## 3. Scope

### In scope
- Monocular camera input
- IMU input
- Monocular VO first
- Filter-style VIO after VO baseline works
- Offline dataset runner
- ROS 2 wrapper for communication and publication
- Diagnostics and status reporting

### Out of scope for early phases
- Loop closure
- Global SLAM
- Relocalization
- Dense mapping
- Learned visual processing in early versions
- Sliding-window optimization backend

## 4. System assumptions
- Camera intrinsics are known
- Camera-IMU extrinsics are known or provided
- Image and IMU timestamps are available
- Monocular vision alone is not metric
- IMU is required for metric scale in VIO mode
- Core library should remain ROS-independent

## 5. Architecture overview

```text
Inputs
  Camera frames
  IMU samples

↓
Visual processing
  v1: Shi-Tomasi + KLT
  v2: ORB option

↓
Motion geometry
  Essential matrix / recover pose / triangulation
  Later: PnP once landmarks exist

↓
State estimator
  VO first
  Filter-style VIO next

↓
Outputs
  Pose
  Linear velocity
  Angular velocity (derived from gyro - bias in VIO)
  Covariance / confidence
  Diagnostics / status

↓
Adapters
  Offline runner
  ROS 2 wrapper
```

## 6. Major modules

### 6.1 Input handling
Responsibilities:
- accept image frames
- accept IMU samples
- validate timestamps and ordering
- buffer IMU samples for frame intervals

### 6.2 Visual processing
Responsibilities:
- detect visual features
- track or match them between frames
- reject weak or invalid correspondences

Planned visual processing modes:
- Phase 1: Shi-Tomasi + KLT
- Later: ORB as an alternative visual processing mode

### 6.3 Motion geometry
Responsibilities:
- undistort / normalize points
- estimate essential matrix with RANSAC
- recover relative pose
- triangulate points
- later use PnP when stable landmarks exist

### 6.4 Initialization
Responsibilities:
- determine whether enough motion exists to initialize
- bootstrap initial state
- in VIO mode, estimate scale / gravity / initial biases sufficiently to begin tracking

### 6.5 State estimator
Chosen direction:
- filter-style VIO

Responsibilities:
- maintain state estimate
- propagate with IMU in VIO mode
- correct with visual measurements
- maintain covariance
- output current estimate and runtime status

### 6.6 Diagnostics
Responsibilities:
- track counts and quality metrics
- expose state/status such as Tracking / Degraded / Lost
- support debugging and benchmarking

### 6.7 Adapters
#### Offline runner
- replay data without ROS
- save trajectory and diagnostics to disk

#### ROS 2 wrapper
- subscribe to ROS topics
- convert ROS messages to core types
- publish odometry / TF / diagnostics

## 7. State definition (VIO target state)
Nominal VIO state:

```text
[p_w_b, q_w_b, v_w_b, b_a, b_g]
```

Error-state covariance ordering:

```text
[δp, δθ, δv, δb_a, δb_g]
```

Notes:
- Linear velocity is part of the state
- Angular velocity is derived from corrected gyro measurements, not stored as a separate state variable
- In Phase 1 VO-only mode, translation scale is arbitrary

## 8. Outputs
Core outputs should eventually include:
- pose estimate
- linear velocity estimate
- covariance
- validity flag
- diagnostics/status

ROS wrapper should later publish:
- `nav_msgs/Odometry`
- TF
- diagnostics

For odometry twist later:
- linear velocity comes from the state estimator
- angular velocity comes from `gyro_measurement - gyro_bias`

## 9. Runtime statuses
Current planned statuses:
- WaitingForData
- Initializing
- Tracking
- Degraded
- Lost

## 10. Development phases

### Phase 1 - Monocular VO with KLT
- Shi-Tomasi detection
- KLT tracking
- Essential matrix + pose recovery
- Triangulation
- Basic VO estimate
- Offline runner first

### Phase 2 - Add IMU and become VIO
- IMU buffering
- IMU-based propagation
- Visual-inertial initialization
- Filter-style correction/update
- Velocity / bias / covariance handling
- Odometry-quality outputs

### Phase 3 - Add ORB as alternate visual processing mode
- ORB detection / description / matching
- Compare against KLT-based visual processing
- Visual processing mode selection via config

### Phase 4 - Robustness and tooling
- Better degraded/lost handling
- Confidence metrics
- Benchmark tooling
- ROS wrapper polish

## 11. Phase order rationale
Chosen order:
1. VO with KLT
2. Add IMU and become VIO
3. Add ORB as alternate visual processing mode
4. Improve robustness and tooling

Reason:
- IMU changes the architecture much more than ORB does
- ORB is mostly a visual processing swap/addition
- IMU affects state, propagation, initialization, outputs, covariance, and timing

## 12. Public API structure
Locked public include directory:

```text
core/include/vio/
├── vio_estimator.hpp
├── config.hpp
├── types.hpp
├── diagnostics.hpp
└── enums.hpp
```

Design rule:
- The public API must not assume KLT forever
- Config and interfaces should allow multiple visual processing modes later

## 13. Current implementation status
Completed or decided so far:
- repo skeleton created
- build baseline working
- public include structure agreed
- `types.hpp` implemented
- visual processing direction chosen: KLT first, ORB later
- VIO family chosen: filter-style
- OpenCV accepted for optimized low-level operations

## 14. Open technical decisions still remaining
These need lower-level design later:
- exact filter formulation
- exact visual update model inside the filter
- exact VIO initialization algorithm
- landmark/map management policy
- whether PnP enters at the end of Phase 1 or later
- confidence model design

## 15. Immediate next step
Create a module-by-module implementation plan for Phase 1 only, with exact files, classes, responsibilities, and milestone order.
