import numpy as np

def calculate_repeatability(poses):
    """
    로봇의 도착 위치 및 방향(x, y, yaw) 데이터 리스트를 사용하여
    위치 및 방향 반복정밀도를 계산합니다. (ISO 9283 기반)

    :param poses: [(x1, y1, yaw1), (x2, y2, yaw2), ...] 형태의 데이터 리스트
    :return: (위치 반복정밀도, 방향 반복정밀도) 튜플
    """
    if len(poses) < 2:
        return 0.0, 0.0

    # 데이터를 numpy 배열로 변환
    data = np.array(poses)
    positions = data[:, :2]  # x, y 위치 데이터 추출
    orientations = data[:, 2] # yaw 방향 데이터 추출

    # 1. 위치 반복정밀도 (RP_L) 계산
    # 각 x, y의 평균값 계산
    mean_position = np.mean(positions, axis=0)
    
    # 각 지점과 평균 지점 사이의 유클리드 거리 계산
    distances = np.linalg.norm(positions - mean_position, axis=1)
    
    # 거리들의 평균값과 표준편차 계산
    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    
    # 위치 반복정밀도: 평균 거리 + 3 * 표준편차
    positional_repeatability = mean_distance + 3 * std_distance

    # 2. 방향 반복정밀도 (RP_O) 계산
    # 각 yaw의 평균과 표준편차 계산
    # 입력된 yaw가 degree 값이면, 결과인 std_orientation도 degree 단위가 됩니다.
    mean_orientation = np.mean(orientations)
    std_orientation = np.std(orientations)
    
    # 방향 반복정밀도: 3 * 표준편차
    orientation_repeatability = 3 * std_orientation
    
    return positional_repeatability, orientation_repeatability

# --- 사용 예시 ---
if __name__ == "__main__":
    # 로봇이 동일한 목표 지점에 10번 도착했을 때의 데이터라고 가정
    # 단위: 미터(m), 도(degrees)
    sample_poses = [
        (-14.5, -6.6, 180.1),
        (-7.3, -6.3, 175.9),
        (-1.5, -33.1, 176.7),
        (-3.9, -30.1, 176.8),
        (-5.0, -12.9, 177.0),
    ]

    rp_l, rp_o = calculate_repeatability(sample_poses)

    print("--- 반복정밀도 계산 결과 ---")
    print(f"측정 횟수: {len(sample_poses)}회")
    # 위치 반복정밀도는 mm로 변환하여 출력
    print(f"위치 반복정밀도 (RP_L): {rp_l:.4f} mm")
    
    # ######################################################
    # ## 여기가 수정된 부분입니다.
    # ## rp_o가 이미 degree 단위이므로 np.rad2deg()를 제거합니다.
    # ######################################################
    print(f"방향 반복정밀도 (RP_O): {rp_o:.4f} degrees")