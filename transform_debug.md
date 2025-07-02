문제의 근본 차이 정리
1. 좌표계 체인(Transformation chain)  
   • Python 코드는 LiDAR 포인트와 ‘콘’ 데이터를 구분해 변환한다.  
     - LiDAR 클라우드 (raw) → T_lidar_to_cam  
     - Sensor (os_sensor) 콘, processed cloud → T_sensor_to_cam = T_lidar_to_cam @ T_sensor_to_lidar  
   • C++ `ProjectionDebugNode`는 항상 하나의 `extrinsic_matrix`(LiDAR→Cam)만 사용한다.  
     → 콘 데이터가 sensor 프레임(= os_sensor)라면 필요한 축반전·이동이 빠져서 좌·우, 상·하가 뒤집히거나 카메라 뒤로 가게 됨.

2. 축 반전‧오프셋 누락의 영향  
   - Python 상수 `T_sensor_to_lidar`  
   ```203:206:ros2_camera_lidar_fusion/ros2_camera_lidar_fusion/project_dual_cameras_points.py
   self.T_sensor_to_cam = self.T_lidar_to_cam @ self.T_sensor_to_lidar
   ```  
   는 X・Y 축 부호를 뒤집고 Z-축을 −0.038 m 내려 준다.  
   - C++ 측에는 동일한 행렬이 존재하지 않아 결과가 이미지 경계 밖이나 음수 Z 로 투영됨.

3. 인덱스 보존 여부  
   - Python: `filter_points_in_front()`가 유효 인덱스를 함께 반환해 색상/레이블을 유지함  
   ```28:63:ros2_camera_lidar_fusion/ros2_camera_lidar_fusion/project_dual_cameras_points.py
   return valid_points, valid_indices
   ```  
   - C++: `projectLidarToCamera()`에서 Z<0 포인트를 버리지만 인덱스는 버리지 않음  
   ```1:34:calico/src/utils/projection_utils.cpp
   if (pt.z > 1e-3) { cv_points.emplace_back(...); }
   ```  
   그 결과 유효-포인트 개수 ≠ 콘 개수 → draw 루프에서 색상/텍스트가 틀어진다.

4. 사소한 차이  
   • 필터링 임계값: Python 0 m, C++ 1 mm – 영향 미미  
   • 프로젝션 함수: Python은 dist_coeffs==0 이면 수식 계산, C++은 `cv::projectPoints` 고정 – 기능은 동일

결론 및 권장 수정
1. C++ 쪽에도 `T_sensor_to_lidar`(또는 그 역행렬)​을 포함해  
   ```cpp
   Eigen::Matrix4d T_sensor_to_lidar = ...;      // 축 반전 + z offset
   Eigen::Matrix4d T_sensor_to_cam   = extrinsic_matrix_ * T_sensor_to_lidar;
   ```  
   를 적용한 뒤 센서계 데이터(콘)에는 `T_sensor_to_cam`을, LiDAR raw 클라우드에는 기존 `extrinsic_matrix_`를 사용하도록 분기한다.

2. `ProjectionUtils::projectLidarToCamera()`에서 Z-필터링 결과에 대응하는 인덱스 배열을 반환하거나,  
   `ProjectionDebugNode`에서 Python과 동일하게 `valid_indices`를 관리해 색상/텍스트가 어긋나지 않도록 한다.

3. 이상이 반영되면 Python 결과와 시각적·수치적으로 일치할 것이다.

## 구체적인 수정 계획 (2025-07-02)

### 1. T_sensor_to_lidar 행렬 추가
ProjectionDebugNode에 Python과 동일한 변환 행렬 추가:
```cpp
// From Ouster OS1 documentation: lidar_to_sensor_transform
Eigen::Matrix4d T_sensor_to_lidar;
T_sensor_to_lidar << -1,  0,  0,  0,
                       0, -1,  0,  0,
                       0,  0,  1, -0.038195,
                       0,  0,  0,  1;
```

### 2. 콘 데이터 변환 체인 수정
sorted_cones_time 토픽의 데이터는 os_sensor 프레임이므로:
```cpp
// 콘 데이터 변환 (os_sensor → camera)
Eigen::Matrix4d T_sensor_to_cam = extrinsic_matrix_ * T_sensor_to_lidar;
// extrinsic_matrix_는 T_lidar_to_cam
```

### 3. ProjectionUtils 수정
- projectLidarToCamera 함수가 유효한 포인트의 인덱스를 반환하도록 수정
- 또는 ProjectionDebugNode에서 직접 필터링과 인덱스 관리

### 4. 예상 결과
- 콘이 올바른 위치에 투영됨 (이미지 경계 내부)
- Valid 개수가 0이 아닌 적절한 값
- 각 콘의 색상과 인덱스가 올바르게 표시됨

https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html

Lidar Range Data To Sensor XYZ Coordinate Frame
For applications that require calibration against a precision mount or use the IMU data (Inertial Measurement Unit) in combination with the lidar data, the XYZ points should be adjusted to the Sensor Coordinate Frame. This requires a Z translation and a rotation of the X,Y,Z points about the Z-axis. The Z translation is the height of the lidar aperture stop above the sensor origin, which varies depending on the sensor you have, and the data must be rotated 180° around the Z-axis. This information can be queried over TCP in the form of a homogeneous transformation matrix in row-major ordering.

Example JSON formatted query using the HTTP command GET /api/v1/sensor/metadata/lidar_intrinsics:

{
  "lidar_to_sensor_transform": [-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 38.195, 0, 0, 0, 1]
}
Which corresponds to the following matrix

\texttt{M\_lidar\_to\_sensor} = \begin{bmatrix}
-1 & 0 & 0 & 0\\
0 & -1 & 0 & 0\\
0 & 0 & 1 & 38.195\\
0 & 0 & 0 & 1
\end{bmatrix}


