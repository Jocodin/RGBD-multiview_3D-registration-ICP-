import numpy as np
import open3d as o3d

# ICP 수행
cap_num = int(input("Capture numbers : "))
icp_num = int(input("ICP numbers : "))
for i in range(cap_num-1):
    print("in progress : " + str(i + 1) + "/" + str(cap_num - 1))
    b_pcd = o3d.io.read_point_cloud("fin_pc/"+str(i+2)+".pcd")
    a_pcd = o3d.io.read_point_cloud("fin_pc/"+str(i+1)+".pcd")
    b_pc = np.asarray(b_pcd.points)
    a_pc = np.asarray(a_pcd.points)

    b_pc__ = []
    a_pc__ = []
    for i1 in range(b_pc.shape[0]): # 최종적으로 사용할 점 선택 (적절히 비율 조절)
        if i1%1000 == 0:
            b_pc__.append(b_pc[i1])
    for i1 in range(a_pc.shape[0]):
        if i1%10 == 0:
            a_pc__.append(a_pc[i1])

    R = np.identity(n=3)
    t = np.array([0,0,0])
    for j in range(icp_num):
        b_pt_ = []
        a_pt_ = []
        for k in range(len(b_pc__)):
            s = None
            s_i = None
            for l in range(len(a_pc__)):
                s_ = ((b_pc__[k][0] - a_pc__[l][0]) ** 2 + (b_pc__[k][1] - a_pc__[l][1]) ** 2 + (
                            b_pc__[k][2] - a_pc__[l][2]) ** 2) ** 0.5
                if s == None or s_ < s:
                    s = s_
                    s_i = l

            if 10 <= s < 20: # 주요 포인트에 대해서 ICP 계산 (적절한 범위 설정)
                a_pt_.append(a_pc__[s_i])
                b_pt_.append(b_pc__[k])


        # 최소자승법 변환행렬 계산 (svd)
        A = np.array(b_pt_)
        B = np.array(a_pt_)
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R_ = np.dot(Vt.T, U.T)
        t_ = centroid_B.T - np.dot(R_, centroid_A.T)
        R = R_ @ R
        t = np.array([(t[0]+t_[0]), (t[1]+t_[1]), (t[2]+t_[2])])

        for k1 in range(len(b_pc__)):
            p1 = R @ np.array([[b_pc__[k1][0]], [b_pc__[k1][1]], [b_pc__[k1][2]]])
            p2 = R @ p1
            p3 = [p2[0][0] + t[0], p2[1][0] + t[1], p2[2][0] + t[2]]
            b_pc__[k1] = np.array(p3)


        # 기존 rt 파일 수정
        ori_R = np.load('rt/'+str(i+2)+'_'+str(i+1)+'_R.npy')
        ori_t = np.load('rt/' + str(i + 2) + '_' + str(i + 1) + '_t.npy')
        res_R = R @ ori_R
        res_t = np.array([(t[0]+ori_t[0]), (t[1]+ori_t[1]), (t[2]+ori_t[2])])
        np.save('rt/'+str(i+2)+'_'+str(i+1)+'_R.npy', res_R)
        np.save('rt/' + str(i + 2) + '_' + str(i + 1) + '_t.npy', res_t)
