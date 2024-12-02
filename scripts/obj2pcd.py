import open3d as o3d
import numpy as np

def convert_obj_to_pcd(obj_file_path, pcd_file_path):
    # .obj 파일 읽기
    mesh = o3d.io.read_triangle_mesh(obj_file_path)
    
    # 메쉬를 포인트 클라우드로 변환
    point_cloud = mesh.sample_points_uniformly(number_of_points=300000)  # 원하는 포인트 수로 샘플링

    # X축을 기준으로 -90도 회전 변환 행렬 생성
    R = point_cloud.get_rotation_matrix_from_xyz((np.radians(90), 0, 0))
    point_cloud.rotate(R, center=(0, 0, 0))

    # .pcd 파일로 저장
    o3d.io.write_point_cloud(pcd_file_path, point_cloud)
    print(f"Converted {obj_file_path} to {pcd_file_path}")

def filter_pcd_by_xz(pcd_file_path, filtered_pcd_file_path, x_min=-0.4, x_max=0.4, y_max=0.5, z_threshold=0.15):
    # .pcd 파일 불러오기
    point_cloud = o3d.io.read_point_cloud(pcd_file_path)
    
    # NumPy 배열로 변환하여 X와 Z 조건으로 필터링
    points = np.asarray(point_cloud.points)
    filtered_points = points[
        (points[:, 0] >= x_min) &  # X가 x_min 이상인 포인트만 선택
        (points[:, 0] <= x_max) &  # X가 x_max 이하인 포인트만 선택
        (points[:, 1] <= y_max) &  # Y가 y_max 이하인 포인트만 선택
        (points[:, 2] <= z_threshold)  # Z가 z_threshold 이상인 포인트만 선택
    ]
    
    # 필터링된 포인트를 새로운 포인트 클라우드로 생성
    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    
    # 필터링된 포인트 클라우드를 새로운 .pcd 파일로 저장
    o3d.io.write_point_cloud(filtered_pcd_file_path, filtered_point_cloud)
    print(f"Filtered points saved to {filtered_pcd_file_path}")
    
    # X, Y, Z 좌표의 평균값 계산
    x_mean = np.mean(filtered_points[:, 0])
    y_mean = np.mean(filtered_points[:, 1])
    z_mean = np.mean(filtered_points[:, 2])
    
    # X, Y, Z 좌표의 최소값과 최대값 계산
    x_min_val = np.min(filtered_points[:, 0])
    x_max_val = np.max(filtered_points[:, 0])
    y_min_val = np.min(filtered_points[:, 1])
    y_max_val = np.max(filtered_points[:, 1])
    z_min_val = np.min(filtered_points[:, 2])
    z_max_val = np.max(filtered_points[:, 2])

    # 필터링된 포인트의 총 개수
    num_points = filtered_points.shape[0]
    
    # 결과 출력
    print(f"Average X: {x_mean:.4f}, Min X: {x_min_val:.4f}, Max X: {x_max_val:.4f}")
    print(f"Average Y: {y_mean:.4f}, Min Y: {y_min_val:.4f}, Max Y: {y_max_val:.4f}")
    print(f"Average Z: {z_mean:.4f}, Min Z: {z_min_val:.4f}, Max Z: {z_max_val:.4f}")
    print(f"Total number of points: {num_points}")

# 파일 경로 설정
# obj_file = "/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro2.obj"
# pcd_file = "/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro2.pcd"
# filtered_pcd_file = "/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro2_filtered.pcd"
obj_file = "/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro_stl.obj"
pcd_file = "/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro_stl.pcd"
filtered_pcd_file = "/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro_stl_filtered.pcd"

# 변환 실행
convert_obj_to_pcd(obj_file, pcd_file)

# X와 Z 조건에 맞춰 포인트를 필터링하여 새로운 파일로 저장 및 통계 출력
# 예시: X는 -0.5 ~ 0.5 범위, Z는 -0.5 이상인 포인트만 남김
filter_pcd_by_xz(pcd_file, filtered_pcd_file, x_min=-0.5, x_max=0.5, y_max=0.7, z_threshold=0.0)
