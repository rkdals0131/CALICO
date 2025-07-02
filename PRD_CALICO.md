# CALICO - 현재 구현 상태 및 개발 계획

## 프로젝트 개요
CALICO는 Python 기반 hungarian_association을 C++로 재구현한 고성능 센서 융합 시스템입니다.
- **목적**: 실시간 콘 검출/추적을 위한 YOLO + LiDAR 융합
- **대상**: 자율주행 레이싱 차량
- **진행률**: 88% 완료

## 현재 구현된 기능

### ✅ 센서 융합 (95% 완료)
- **멀티카메라 지원**: 동시 다중 카메라 처리
- **LiDAR-카메라 투영**: 3D→2D 변환 및 캘리브레이션
- **Hungarian 매칭**: dlib 기반 구현 (정방 행렬 자동 패딩)
- **색상 충돌 해결**: 투표 기반 색상 결정

### ✅ 추적 시스템 (60% 완료)
- **UKF 추적기**: 기본 구현 (개선 필요)
- **IMU 보상**: EMA/Butterworth 필터
- **트랙 관리**: 생성/업데이트/삭제 로직
- **색상 히스토리**: 시간 기반 투표

### ✅ 시각화 및 디버깅 (100% 완료)
- **RViz 마커**: 색상별 콘 시각화
- **투영 디버그 노드**: 카메라 이미지에 LiDAR 오버레이
- **Launch 파일**: 개별/통합 실행 지원

### ✅ 설정 관리 (100% 완료)
- **Python 호환**: 기존 YAML 설정 파일 재사용
- **동적 파라미터**: ROS2 파라미터 서버 지원

## 앞으로 개발할 것

### 1. 주파수 및 안정성 문제 해결 (긴급)
- [ ] 15Hz → 50Hz 주파수 불일치 원인 파악
- [ ] 트랙 ID 빠른 갱신 문제 해결
- [ ] 색상 유지 안되는 문제 디버깅

### 2. UKF 완전 구현 (1주)
- [ ] Python filterpy와 동일한 동작 구현
- [ ] Sigma point 생성/전파 개선
- [ ] 파라미터 튜닝 (R=0.1, max_age=4)
- [ ] 또는 외부 라이브러리 도입

### 3. 성능 최적화 (3일)
- [ ] 프로파일링 및 병목 지점 파악
- [ ] TBB 기반 병렬 처리
- [ ] 메모리 풀 구현
- [ ] SIMD 최적화

### 4. 테스트 및 검증 (3일)
- [ ] Python 출력과 1:1 비교
- [ ] 실시간 성능 측정 (목표: <10ms/frame)
- [ ] 장시간 안정성 테스트
- [ ] 메모리 누수 검사

### 5. 코드 품질 (2일)
- [ ] 단위 테스트 작성 (gtest)
- [ ] 컴파일 경고 제거
- [ ] Doxygen 문서화

## 기술 스택
- **C++17**: 현대적 C++ 기능 활용
- **Eigen3**: 행렬 연산
- **OpenCV**: 카메라 투영
- **dlib**: Hungarian 알고리즘 (max_cost_assignment)
- **yaml-cpp**: 설정 파일 파싱
- **ROS2 Humble**: 미들웨어

## 입출력 인터페이스

### 입력 토픽
- `/sorted_cones_time`: LiDAR 콘 (os_sensor 프레임)
- `/camera_{1,2}/detections`: YOLO 검출 결과
- `/ouster/imu`: IMU 데이터 (선택)

### 출력 토픽
- `/fused_sorted_cones`: 색상 정보가 추가된 콘
- `/fused_sorted_cones_ukf`: 추적된 콘 (ID 포함)
- `/visualization_marker_array`: RViz 시각화

## 빌드 및 실행

```bash
# 빌드
colcon build --packages-select calico

# 실행
ros2 launch calico calico_full.launch.py

# 디버그 모드
ros2 launch calico calico_full.launch.py enable_debug_viz:=true
```

## 알려진 이슈
1. **주파수 불일치 해결 완료**: ✅
   - 원인: 각 메시지마다 tryFusion() 호출되던 문제
   - 해결: ApproximateTimeSynchronizer 구현으로 동기화된 콜백만 실행

2. **트랙 ID 불안정**: 칼만필터 트랙 ID가 빠르게 갱신됨
   - 원인: 높은 주파수로 인한 과도한 업데이트
   - 계획: 주파수 문제 해결 후 재확인

2. **UKF 기능 제한**: filterpy 대비 기본 기능만 구현
   - 현재: 기본 예측/업데이트만 동작
   - 계획: 완전 구현 또는 라이브러리 도입

3. **Butterworth 필터**: 간소화된 2차 구현
   - 현재: 기본 필터링만 지원
   - 계획: DSP 라이브러리 도입 검토

## 성능 목표
- **처리 속도**: < 10ms/frame (단일 카메라)
- **메모리 사용**: < 500MB
- **매칭 정확도**: > 95%
- **Python 대비**: 5배 이상 빠른 처리

## 연락처
- 유지보수: kikiws70@gmail.com
- 이슈 제보: https://github.com/anthropics/claude-code/issues