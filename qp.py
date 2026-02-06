#!/usr/bin/env python3
import numpy as np
import cvxpy as cp


#在使用之前，建议去学习Control barrier function(CBF)相关的知识


#1、函数区别：
#以下共有两个函数，选择其中一个使用即可。
#第一个为纯CBF修正，很容易陷入死锁问题。第二个为CBF修正之前加入一个偏移量，可以很大程度上避免陷入死锁问题。

#2、使用方法：
#我使用激光雷达获取到最近障碍物的坐标点,再根据无人机当前坐标点和最近障碍物坐标点得出d_x和d_y。

#3、函数输入值定义：
#d_x,d_y为无人机和障碍物的x方向和y轴方向的距离，需要实时变化;  min_dist为自己设定的最小安全距离，固定值(比如0.5m),v_y为无人机当前在x，y轴方向的速度，需要实时变化





# def qp_control(d_x,d_y,min_dist,v_x,v_y):

#     u = cp.Variable(2)
#     u_nom = cp.Parameter(2)
#     u_nom.value = np.array([v_x, v_y])

#     objective = cp.Minimize(0.5 * cp.sum_squares(u - u_nom))

#     # constrain = [u[0]+u[1] <= 1,
#     #              u[0] <= 0.5,
#     #              u[1] <= 0.5
#     #              ]
 
#     constrain = [2*d_x*u[0]+2*d_y*u[1] >= -((d_x**2)+(d_y**2)-(min_dist**2))*1
                
#                  ]
#     # constrain = [u[0]<=0]



#     prob = cp.Problem(objective,constrain)
#     prob.solve()

#     if prob.status == cp.OPTIMAL:
#         print(f"理想vx vy = [{v_x} {v_y}]")
#         print(f"最优解 u: {u.value}")
#         print(f"最优解cost = {prob.value}\n")
#         return u.value
#     else:
#         print("问题无解")
#         return v_x,v_y

    





def qp_control(d_x, d_y, min_dist, v_x, v_y):
    # 计算距离
    dist = np.sqrt(d_x**2 + d_y**2)
    x = v_x
    y = v_y
    # --- 增加安全性检查，防止 dist 为 0 或过小导致的报错 ---
    if dist > 0.001 and dist < min_dist * 2.0:
        # 1. 计算切方向
        t_x, t_y = -d_y / dist, d_x / dist
        
        # 2. 智能判断方向：确保偏移方向与理想速度一致
        if (t_x * v_x + t_y * v_y) < 0:
            t_x, t_y = -t_x, -t_y
            
        # 3. 加上偏移量 (0.5 是偏移强度)
        v_x += t_x * 0.7
        v_y += t_y * 0.7

    # 再次检查，确保 v_x, v_y 是正常的实数，防止传给 cvxpy 时报错
    if np.isnan(v_x) or np.isnan(v_y):
        v_x, v_y = 0.0, 0.0

    u = cp.Variable(2)
    u_nom = cp.Parameter(2)
    u_nom.value = np.array([float(v_x), float(v_y)]) # 强制转为实数

    objective = cp.Minimize(0.5 * cp.sum_squares(u - u_nom))
    
    # 距离平方计算，保持你原来的约束
    dist_sq = d_x**2 + d_y**2
    h = dist_sq - (min_dist**2)
    
    constrain = [2*d_x*u[0] + 2*d_y*u[1] >= -h]

    prob = cp.Problem(objective, constrain)
    # 使用 OSQP 求解器通常更稳定
    prob.solve()

    if prob.status == cp.OPTIMAL:
        print(f"理想vx vy = [{v_x} {v_y}]")
        print(f"最优解 u: {u.value}")
        print(f"最优解cost = {prob.value}\n")
        return u.value
    else:
        # 如果无解，安全起见返回 0 速度
        return np.array([x, y])#返回原速度








if __name__ == "__main__":
    print("this qp")

