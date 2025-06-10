import numpy as np

# 区域边界定义
BOUNDARY = [(0, 0), (10, 0), (10, 10), (0, 10)]  # 区域边界
START_POINT = (0, 0)                              # 入口点 (heading inward)
START_HEADING = np.pi/4                           # 45度向右上方
END_POINT = (10, 10)                              # 出口点 (heading outward)
END_HEADING = np.pi/4                             # 45度向右上方

# 边界上的途径点 - 包含位置和朝向
# 每个waypoint: (x, y, heading_inward)
WAYPOINTS = [
    (0, 3, 0),          # 左边界，heading向右(0度)
    (3, 10, -np.pi/2),  # 上边界，heading向下(-90度)
    (10, 7, np.pi),     # 右边界，heading向左(180度) 
    (7, 0, np.pi/2)     # 下边界，heading向上(90度)
]

# 车辆运动学参数
MIN_TURNING_RADIUS = 1.0      # 前进最小转弯半径
REVERSE_TURNING_FACTOR = 1.5  # 倒车时转弯半径增大因子
MAX_REVERSE_LENGTH = 3.0      # 最大连续倒车距离

# 进化算法参数
POPULATION_SIZE = 10
MAX_GENERATIONS = 5
CX_PROBABILITY = 0.7
MUT_PROBABILITY = 0.2
TOURNAMENT_SIZE = 3

# 适应度函数权重
WEIGHTS = {
    'crossing': 10.0,       # 路径交叉
    'length': 0.1,          # 总路径长度
    'curvature': 1.0,       # 曲率约束
    'boundary': 10.0,       # 边界违规
    'overlap': 5.0,         # 路径重叠
    'reverse_count': 2.0,   # 倒车段计数
    'reverse_length': 0.5,  # 倒车段长度
}

# 安全距离
SAFE_DISTANCE = 0.5         # 路径最小安全距离

# 倒车配置
REVERSE_ANGLE_TOLERANCE = np.pi/6  # 倒车方向容忍角度(30度)