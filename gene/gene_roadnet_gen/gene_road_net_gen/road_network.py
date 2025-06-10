import numpy as np
import random
from shapely.geometry import LineString, Point, Polygon
from config import *

class ReversePath:
    """表示包含倒车操作的路径段"""
    def __init__(self, points, reverse_flags):
        self.points = points  # 路径点列表[(x, y, heading)]
        self.reverse_flags = reverse_flags  # 布尔列表，True表示倒车
        self.line = self.create_linestring()
    
    def create_linestring(self):
        """创建LineString表示（忽略方向）"""
        return LineString([(p[0], p[1]) for p in self.points])
    
    @property
    def reverse_count(self):
        """计算倒车段数量"""
        count = 0
        in_reverse = False
        
        for flag in self.reverse_flags:
            if flag and not in_reverse:
                count += 1
                in_reverse = True
            elif not flag:
                in_reverse = False
                
        return count
    
    @property
    def reverse_length(self):
        """计算倒车总长度"""
        total = 0.0
        for i in range(len(self.points)-1):
            if self.reverse_flags[i]:
                p1 = np.array(self.points[i][:2])
                p2 = np.array(self.points[i+1][:2])
                total += np.linalg.norm(p2 - p1)
        return total

class RoadNetwork:
    def __init__(self, genes=None):
        """
        路网个体表示 - 倒车版本
        genes: 基因序列(控制点)，如未提供则随机生成
        """
        self.genes = genes if genes else self.generate_random_genes()
        self.path_segments = self.generate_paths()
    
    def generate_random_genes(self):
        """生成随机基因序列"""
        genes = []
        # 每个途径点有两个控制点
        for _ in range(len(WAYPOINTS)):
            # 控制点1 (起点附近)
            x = random.uniform(START_POINT[0], START_POINT[0] + 3)
            y = random.uniform(START_POINT[1], START_POINT[1] + 3)
            genes.append((x, y))
            
            # 控制点2 (终点附近)
            x = random.uniform(END_POINT[0] - 3, END_POINT[0])
            y = random.uniform(END_POINT[1] - 3, END_POINT[1])
            genes.append((x, y))
        return genes
    
    def generate_paths(self):
        """生成完整路径链（含倒车段）"""
        segments = []
        
        # 入口点 → 每个途径点的路径
        for i, waypoint in enumerate(WAYPOINTS):
            ctrl1 = self.genes[i*2]
            segment = self.generate_waypoint_path(START_POINT, waypoint, ctrl1, is_to_waypoint=True)
            segments.append(segment)
        
        # 每个途径点 → 出口点的路径
        for i, waypoint in enumerate(WAYPOINTS):
            ctrl2 = self.genes[i*2+1]
            segment = self.generate_waypoint_path(waypoint, END_POINT, ctrl2, is_to_waypoint=False)
            segments.append(segment)
        
        return segments
    
    def generate_waypoint_path(self, start, end, control_point, is_to_waypoint):
        """
        生成单条路径段（含倒车操作）
        is_to_waypoint: True表示进入途径点，需要倒车
        """
        # 创建三次贝塞尔曲线
        bezier_points = self.cubic_bezier(start, control_point, end, 20)
        headings = self.calculate_headings(bezier_points)
        
        # 识别倒车段
        reverse_flags = [False] * len(bezier_points)
        
        if is_to_waypoint:
            # 计算倒车起始点（接近途径点时）
            reverse_start_idx = int(len(bezier_points) * 0.7)  # 后30%路径
            
            # 确保倒车方向合理
            target_heading = self.calculate_desired_heading(bezier_points[-1], end)
            
            for i in range(reverse_start_idx, len(bezier_points)):
                reverse_flags[i] = True
                
                # 调整倒车段方向以匹配期望方向
                angle_diff = self.angle_difference(headings[i], target_heading)
                if abs(angle_diff) > REVERSE_ANGLE_TOLERANCE:
                    # 倒车时需要更小的转弯半径
                    headings[i] = target_heading
        
        return ReversePath(
            points=[(p[0], p[1], h) for p, h in zip(bezier_points, headings)],
            reverse_flags=reverse_flags
        )
    
    def cubic_bezier(self, p0, p1, p2, num_points):
        """生成三次贝塞尔曲线点"""
        t = np.linspace(0, 1, num_points)
        points = []
        for i in t:
            x = (1-i)**2 * p0[0] + 2*(1-i)*i * p1[0] + i**2 * p2[0]
            y = (1-i)**2 * p0[1] + 2*(1-i)*i * p1[1] + i**2 * p2[1]
            points.append((x, y))
        return points
    
    def calculate_headings(self, points):
        """计算路径点处的航向角"""
        headings = [0.0] * len(points)
        
        for i in range(1, len(points)-1):
            dx = points[i+1][0] - points[i-1][0]
            dy = points[i+1][1] - points[i-1][1]
            headings[i] = np.arctan2(dy, dx)
        
        # 起点和终点航向
        if len(points) > 1:
            dx = points[1][0] - points[0][0]
            dy = points[1][1] - points[0][1]
            headings[0] = np.arctan2(dy, dx)
            
            dx = points[-1][0] - points[-2][0]
            dy = points[-1][1] - points[-2][1]
            headings[-1] = np.arctan2(dy, dx)
        
        return headings
    
    def calculate_desired_heading(self, point, waypoint):
        """计算在途径点处的期望航向（垂直于边界）"""
        # 简化实现：总是垂直于X轴
        # 实际实现应基于边界法向量
        return np.pi / 2  # 垂直向上
    
    def angle_difference(self, a, b):
        """计算两个角度之间的最小差值（-π到π）"""
        diff = b - a
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff