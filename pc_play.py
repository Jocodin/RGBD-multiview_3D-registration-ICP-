import open3d as o3d

cap_num = int(input("Captured numbers : "))

# 포인트클라우드 시각화
pcd_list = []
for i in range(cap_num):
    pcd = o3d.io.read_point_cloud("fin_pc/" + str(i+1) + ".pcd")
    pcd_list.append(pcd)

print("[ Point-cloud Created ]")
o3d.visualization.draw_geometries(pcd_list)