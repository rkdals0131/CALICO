# CALICO (Cone Attribute Linking by Image and Cluster Output) 제품 요구사항 정의서

## 1. 개요

### 1.1 프로젝트 소개
CALICO는 기존 Python 기반 hungarian_association 패키지를 C++로 완전히 재구현하는 프로젝트입니다. 자율주행 차량의 콘(Cone) 감지 및 추적을 위한 고성능 센서 융합 시스템으로, YOLO 객체 검출과 LiDAR 포인트 클라우드 데이터를 결합합니다.

### 1.2 목표
- **성능 향상**: C++ 구현을 통한 실시간 처리 성능 개선
- **메모리 효율성**: Python 대비 메모리 사용량 감소
- **확장성**: 멀티 카메라 및 멀티 센서 지원
- **안정성**: 강건한 타입 시스템과 예외 처리

## 2. 기술 스택

### 2.1 핵심 라이브러리
- **Eigen3**: 행렬 연산 및 선형 대수 ✅
- **OpenCV 4.x**: 카메라 캘리브레이션 및 프로젝션 ✅
- **yaml-cpp**: YAML 설정 파일 파싱 ✅
- **자체 구현**: Hungarian 알고리즘 (간소화 버전) ✅
- **ROS2 (Humble/Iron)**: 미들웨어 및 통신 ✅

### 2.2 추가 고려사항
- **TBB (Threading Building Blocks)**: 병렬 처리 최적화
- **PCL (Point Cloud Library)**: LiDAR 데이터 처리 (선택사항)
- **gtest/gmock**: 단위 테스트

## 3. 아키텍처 설계

### 3.1 모듈 구조
```
calico/
├── include/calico/
│   ├── fusion/
│   │   ├── hungarian_matcher.hpp
│   │   ├── sensor_fusion.hpp
│   │   └── multi_camera_fusion.hpp
│   ├── tracking/
│   │   ├── ukf_tracker.hpp
│   │   ├── track.hpp
│   │   └── imu_compensator.hpp
│   ├── utils/
│   │   ├── config_loader.hpp
│   │   ├── projection_utils.hpp
│   │   └── message_converter.hpp
│   └── visualization/
│       └── rviz_marker_publisher.hpp
├── src/
│   ├── fusion/
│   ├── tracking/
│   ├── utils/
│   ├── visualization/
│   └── nodes/
│       ├── multi_camera_fusion_node.cpp
│       ├── ukf_tracking_node.cpp
│       └── visualization_node.cpp
└── test/
```

### 3.2 핵심 클래스 설계

#### 3.2.1 HungarianMatcher
```cpp
class HungarianMatcher {
public:
    struct MatchResult {
        std::vector<std::pair<int, int>> matches;
        std::vector<int> unmatched_detections;
        std::vector<int> unmatched_tracks;
    };
    
    MatchResult match(const Eigen::MatrixXd& cost_matrix, 
                      double threshold = 50.0);
};
```

#### 3.2.2 UKFTracker
```cpp
class UKFTracker {
private:
    struct TrackState {
        Eigen::VectorXd mean;  // [x, y, vx, vy]
        Eigen::MatrixXd covariance;
        std::deque<std::string> color_history;
        int track_id;
        int age;
    };
    
public:
    void predict(double dt, const IMUData& imu_data);
    void update(const std::vector<Detection>& detections);
    std::vector<TrackedCone> getTracks() const;
};
```

#### 3.2.3 SensorFusion
```cpp
class SensorFusion {
private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    Eigen::Matrix4d T_camera_lidar_;
    
public:
    FusedCones fuse(const LiDARCones& lidar_cones,
                    const YOLODetections& yolo_detections);
};
```

## 4. 기능 요구사항

### 4.1 센서 융합 (Sensor Fusion)
- **FR-1**: LiDAR 3D 포인트를 카메라 2D 평면으로 투영
- **FR-2**: Hungarian 알고리즘을 사용한 YOLO 박스와 투영된 포인트 매칭
- **FR-3**: 매칭된 콘에 YOLO 클래스 레이블 할당
- **FR-4**: 매칭되지 않은 콘은 "Unknown" 레이블 부여

### 4.2 추적 시스템 (Tracking System)
- **FR-5**: Unscented Kalman Filter를 사용한 2D 콘 추적
- **FR-6**: IMU 데이터를 활용한 모션 보상
- **FR-7**: 색상 투표 메커니즘을 통한 안정적인 색상 분류
- **FR-8**: 트랙 생성, 업데이트, 삭제 로직

### 4.3 멀티 카메라 지원
- **FR-9**: 다중 카메라 스트림 동시 처리
- **FR-10**: 카메라별 독립적인 캘리브레이션 파라미터
- **FR-11**: 시간 동기화된 멀티 센서 데이터 처리

### 4.4 설정 관리
- **FR-12**: YAML 기반 런타임 설정
- **FR-13**: ROS2 파라미터 서버를 통한 동적 파라미터 조정
- **FR-14**: 카메라 캘리브레이션 파일 자동 로딩

## 5. 성능 요구사항

### 5.1 처리 속도
- **PR-1**: 단일 카메라 융합: < 10ms/frame
- **PR-2**: 멀티 카메라 융합 (2대): < 20ms/frame
- **PR-3**: UKF 추적 (100개 트랙): < 5ms/update

### 5.2 메모리 사용량
- **PR-4**: 최대 메모리 사용량 < 500MB
- **PR-5**: 트랙당 메모리 오버헤드 < 1KB

### 5.3 정확도
- **PR-6**: 매칭 정확도 > 95%
- **PR-7**: 추적 위치 오차 < 10cm (10m 거리 기준)

## 6. 인터페이스 정의

### 6.1 ROS2 토픽 인터페이스
```yaml
# 입력 토픽
/sorted_cones_time: custom_interface/msg/ModifiedFloat32MultiArray
/detections: yolo_msgs/msg/DetectionArray
/camera_{1,2}/detections: yolo_msgs/msg/DetectionArray
/ouster/imu: sensor_msgs/msg/Imu

# 출력 토픽
/fused_sorted_cones: custom_interface/msg/ModifiedFloat32MultiArray
/fused_sorted_cones_ukf: custom_interface/msg/TrackedConeArray
/visualization_marker_array: visualization_msgs/msg/MarkerArray
```

### 6.2 커스텀 메시지 타입
C++ 구현에 맞춰 재정의 필요:
- `TrackedCone.msg`
- `TrackedConeArray.msg`
- `ModifiedFloat32MultiArray.msg`

## 7. 구현 로드맵

### Phase 1: 기반 구조 (2주) ✅ 완료
- CMakeLists.txt 및 package.xml 설정 ✅
- 기본 디렉토리 구조 생성 ✅
- 외부 라이브러리 통합 ✅
- 유틸리티 클래스 구현 (config_loader, projection_utils) ✅

### Phase 2: 센서 융합 (3주) ✅ 완료
- Hungarian 알고리즘 구현/통합 ✅
- 멀티 카메라 융합 노드 구현 ✅
- 메시지 컨버터 구현 ✅
- 프로젝션 유틸리티 구현 ✅

### Phase 3: 추적 시스템 (3주)
- UKF 추적기 구현
- IMU 보상 로직 구현
- 색상 투표 메커니즘 구현
- 추적 노드 구현

### Phase 4: 시각화 및 최적화 (2주)
- RViz 마커 퍼블리셔 구현
- 성능 프로파일링 및 최적화
- 멀티스레딩 적용
- 메모리 최적화

### Phase 5: 테스트 및 검증 (2주)
- 단위 테스트 작성
- 통합 테스트
- 성능 벤치마크
- Python 구현과의 비교 검증

## 8. 최적화 전략

### 8.1 알고리즘 최적화
- **Hungarian 알고리즘**: O(n³) 복잡도 최적화를 위한 sparse matrix 활용
- **UKF**: Cholesky 분해 캐싱 및 SIMD 명령어 활용
- **프로젝션**: 배치 처리 및 OpenCV 최적화 함수 활용

### 8.2 병렬 처리
- **멀티 카메라 처리**: 카메라별 독립 스레드
- **매칭 및 추적**: TBB parallel_for 활용
- **ROS2 executor**: MultiThreadedExecutor 사용

### 8.3 메모리 최적화
- **Object Pool**: 빈번한 할당/해제 방지
- **Ring Buffer**: 히스토리 데이터 관리
- **Move Semantics**: 불필요한 복사 방지

## 9. 테스트 계획

### 9.1 단위 테스트
- Hungarian 매칭 알고리즘 정확도
- UKF 예측/업데이트 로직
- 투영 변환 정확도
- 설정 파일 파싱

### 9.2 통합 테스트
- 엔드투엔드 데이터 플로우
- 멀티 센서 시간 동기화
- 에러 복구 시나리오
- 성능 스트레스 테스트

### 9.3 검증 기준
- Python 구현 대비 동일한 출력 (오차 < 1%)
- 실시간 처리 요구사항 충족
- 메모리 누수 없음
- 장시간 안정성

## 10. 위험 요소 및 대응 방안

### 10.1 기술적 위험
- **위험**: UKF 수치 안정성 문제
- **대응**: Eigen의 robust decomposition 사용, 상태 정규화

### 10.2 성능 위험
- **위험**: 실시간 처리 목표 미달성
- **대응**: 프로파일링 기반 병목 지점 최적화, GPU 가속 고려

### 10.3 호환성 위험
- **위험**: ROS2 버전 간 API 차이
- **대응**: 조건부 컴파일, 버전별 래퍼 구현

## 11. 유지보수 계획

### 11.1 문서화
- Doxygen 기반 API 문서
- 사용자 가이드 (한국어/영어)
- 개발자 가이드 및 아키텍처 문서

### 11.3 버전 관리
- Semantic Versioning 준수
- Change Log 유지
- 하위 호환성 보장

## 12. 성공 지표

- Python 대비 5배 이상 성능 향상
- 메모리 사용량 50% 감소
- 코드 커버리지 > 80%
- 문서화 완성도 100%
- 실차 테스트 성공률 > 99%

## 13. 현재 진행 상황 (2025-07-01)

### ✅ 완료된 항목

#### 1. **기반 구조**
- 패키지 구조 및 빌드 시스템 ✅
- 설정 파일 로더 (Python 호환) ✅
- 메시지 변환 유틸리티 ✅
- Launch 파일 (개별 및 통합) ✅

#### 2. **센서 융합**
- Hungarian 알고리즘 구현 (간소화 버전) ✅
- 멀티카메라 융합 로직 ✅
- LiDAR-카메라 프로젝션 ✅
- 충돌 해결 메커니즘 (투표 방식) ✅

#### 3. **추적 시스템**
- UKF 추적기 기본 구현 ✅
- IMU 보상기 (EMA/Butterworth 필터) ✅
- 트랙 관리 (생성/업데이트/삭제) ✅
- 색상 히스토리 및 투표 ✅

#### 4. **시각화**
- RViz 마커 퍼블리셔 ✅
- 색상별 콘 시각화 ✅
- 트랙 ID 표시 ✅

### 🚧 진행 중/개선 필요 항목

#### 1. **알고리즘 개선**
- **Hungarian 알고리즘**: 현재 greedy approach → 완전한 O(n³) 구현 필요
  - 옵션: dlib::max_cost_assignment 또는 munkres-cpp 라이브러리
- **UKF 구현**: Python의 filterpy 대체 구현 필요
  - 현재: 기본적인 sigma point 생성/예측/업데이트
  - 필요: 완전한 UKF 구현 또는 외부 라이브러리 (예: kalman-cpp)

#### 2. **필터 구현**
- **Butterworth 필터**: scipy.signal.butter 대체
  - 현재: 간소화된 2차 필터
  - 필요: 완전한 필터 계수 계산 또는 DSP 라이브러리 사용

### 📋 TODO 리스트

#### 1. **디버깅 도구** ✅
- **투영 시각화 노드**: LiDAR 점들을 카메라 이미지에 오버레이
  - 입력: /sorted_cones_time, /camera_1/image_raw
  - 처리: LiDAR 점을 이미지 평면에 투영
  - 출력: /debug/projection_overlay (초록색 점으로 표시)
  - 상태: **구현 완료** (projection_debug_node)

#### 2. **라이브러리 통합**
- Hungarian 알고리즘 라이브러리 선정 및 통합
- UKF 라이브러리 조사 (또는 Eigen 기반 완전 구현)
- DSP 라이브러리 (필터 설계용)

#### 3. **성능 최적화**
- 프로파일링 및 병목 지점 파악
- 병렬 처리 구현 (TBB 또는 std::execution)
- 메모리 풀 구현

#### 4. **테스트 및 검증**
- 단위 테스트 작성
- Python 구현과의 출력 비교
- 실시간 성능 벤치마킹

### ⚠️ Python과의 주요 차이점

| 기능 | Python | C++ (CALICO) | 상태 |
|------|--------|--------------|------|
| Hungarian 알고리즘 | scipy.optimize.linear_sum_assignment | 간소화된 greedy 구현 | 개선 필요 |
| UKF | filterpy.kalman.UnscentedKalmanFilter | 기본 구현 | 개선 필요 |
| Butterworth 필터 | scipy.signal.butter | 간소화된 2차 필터 | 개선 필요 |
| 행렬 연산 | NumPy | Eigen | ✅ |
| YAML 파싱 | PyYAML | yaml-cpp | ✅ |
| 이미지 처리 | OpenCV (Python) | OpenCV (C++) | ✅ |

### 🔍 디버깅 현황

#### 매칭 0 문제 가능 원인:
1. 카메라 캘리브레이션 파라미터 문제
2. YOLO 검출이 없음
3. 투영된 점들이 이미지 범위를 벗어남
4. 좌표계 변환 문제

#### 해결 방안:
1. ✅ 투영 시각화 노드 구현으로 실제 투영 위치 확인 (완료)
2. 각 단계별 로그 추가
3. Python 구현과 중간 결과 비교

### 📊 진행 상황 요약 (2025-07-01)

| 구성 요소 | 구현 상태 | 품질 | 비고 |
|-----------|-----------|------|------|
| **패키지 구조** | ✅ 완료 | 100% | CMakeLists.txt, package.xml |
| **Config 로더** | ✅ 완료 | 100% | Python 호환 |
| **Hungarian 매칭** | ✅ 완료 | 30% | Greedy 알고리즘 (개선 필요) |
| **멀티카메라 융합** | ✅ 완료 | 80% | 기본 기능 구현 |
| **UKF 추적** | ✅ 완료 | 50% | 기본 구현 (개선 필요) |
| **IMU 보상** | ✅ 완료 | 70% | EMA/Butterworth 필터 |
| **시각화** | ✅ 완료 | 100% | RViz 마커 |
| **Launch 파일** | ✅ 완료 | 100% | 개별 및 통합 |
| **디버그 도구** | ✅ 완료 | 100% | 투영 시각화 노드 |

**전체 진행률**: 약 80% (기능 구현 완료, 알고리즘 최적화 필요)