import open3d as o3d

# 读取点云
pcd = o3d.io.read_point_cloud("scan_result.pcd")

# 可视化
o3d.visualization.draw_geometries([pcd], window_name="Point Cloud Viewer")
