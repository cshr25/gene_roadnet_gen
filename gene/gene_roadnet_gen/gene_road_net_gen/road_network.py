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
        """
        生成随机基因序列
        新基因结构：每个途径点有4个参数
        [ctrl1_x, ctrl1_y, turn_distance, turn_angle, ctrl2_x, ctrl2_y, ...]
        """
        genes = []
        # 每个途径点有4个基因参数
        for _ in range(len(WAYPOINTS)):
            # 控制点1 (起点附近) - 用于到达途径点的前进路径
            x1 = random.uniform(START_POINT[0], START_POINT[0] + 3)
            y1 = random.uniform(START_POINT[1], START_POINT[1] + 3)
            genes.extend([x1, y1])
            
            # 转向点参数 - 用于倒车操作
            turn_distance = random.uniform(TURN_DISTANCE_MIN, TURN_DISTANCE_MAX)
            turn_angle = random.uniform(-TURN_ANGLE_MAX, TURN_ANGLE_MAX)
            genes.extend([turn_distance, turn_angle])
            
            # 控制点2 (终点附近) - 用于离开途径点的前进路径
            x2 = random.uniform(END_POINT[0] - 3, END_POINT[0])
            y2 = random.uniform(END_POINT[1] - 3, END_POINT[1])
            genes.extend([x2, y2])
        return genes
    
    def generate_paths(self):
        """
        生成完整路径链（含倒车段）
        新基因结构：每个途径点有6个基因 [ctrl1_x, ctrl1_y, turn_distance, turn_angle, ctrl2_x, ctrl2_y]
        """
        segments = []
        
        # 入口点 → 每个途径点的路径
        for i, waypoint in enumerate(WAYPOINTS):
            gene_offset = i * 6  # 每个途径点6个基因参数
            ctrl1 = (self.genes[gene_offset], self.genes[gene_offset + 1])
            turn_distance = self.genes[gene_offset + 2]
            turn_angle = self.genes[gene_offset + 3]
            
            # 创建转向参数字典
            turn_params = {
                'distance': turn_distance,
                'angle': turn_angle
            }
            
            # 从起点出发，目标是途径点，包含转向参数
            segment = self.generate_waypoint_path(START_POINT, START_HEADING, waypoint, ctrl1, turn_params, is_to_waypoint=True)
            segments.append(segment)
        
        # 每个途径点 → 出口点的路径
        for i, waypoint in enumerate(WAYPOINTS):
            gene_offset = i * 6
            ctrl2 = (self.genes[gene_offset + 4], self.genes[gene_offset + 5])
            
            # 从途径点出发，使用途径点的朝向，目标是END_POINT + END_HEADING
            end_with_heading = (END_POINT[0], END_POINT[1], END_HEADING)
            segment = self.generate_waypoint_path(waypoint, waypoint[2], end_with_heading, ctrl2, None, is_to_waypoint=False)
            segments.append(segment)
        
        return segments
    
    def generate_waypoint_path(self, start_pos, start_heading, end_waypoint, control_point, turn_params, is_to_waypoint):
        """
        生成真实的车辆路径（含实际倒车操作）
        start_pos: 起始位置 (x, y) 或 (x, y, heading)
        start_heading: 起始朝向
        end_waypoint: 目标途径点 (x, y, heading_inward)
        control_point: 路径控制点 (x, y)
        turn_params: 转向参数字典 {'distance': float, 'angle': float} 或 None
        is_to_waypoint: True表示进入途径点，需要倒车
        """
        if isinstance(start_pos, tuple) and len(start_pos) == 3:
            start_pos = (start_pos[0], start_pos[1])
        
        end_pos = (end_waypoint[0], end_waypoint[1])
        end_heading = end_waypoint[2]
        
        if is_to_waypoint:
            # 进入途径点：需要倒车操作，使用进化的转向参数
            return self.generate_backing_maneuver(start_pos, start_heading, end_pos, end_heading, control_point, turn_params)
        else:
            # 离开途径点：正常前进
            return self.generate_forward_path(start_pos, start_heading, end_pos, END_HEADING, control_point)
    
    def generate_backing_maneuver(self, start_pos, start_heading, end_pos, end_heading, control_point, turn_params):
        """
        生成倒车操作路径：前进到位置 -> 停止转向 -> 倒车到位
        turn_params: 进化的转向参数 {'distance': float, 'angle': float}
        """
        all_points = []
        all_reverse_flags = []
        
        # Step 1: 前进到倒车起始点 - 使用进化的参数
        # 计算倒车起始点：使用进化的距离和角度偏移
        backing_distance = turn_params['distance']
        angle_offset = turn_params['angle']
        
        # 基础倒车方向 + 角度偏移
        backing_angle = end_heading + angle_offset
        reverse_start_x = end_pos[0] - backing_distance * np.cos(backing_angle)
        reverse_start_y = end_pos[1] - backing_distance * np.sin(backing_angle)
        reverse_start_pos = (reverse_start_x, reverse_start_y)
        
        # 前进段：从起点到倒车起始点
        # 倒车时的车头朝向：要与倒车移动方向相反
        reverse_heading_target = backing_angle + np.pi
        forward_points = self.create_smooth_path(start_pos, start_heading, reverse_start_pos, reverse_heading_target, control_point, 15)
        forward_headings = self.calculate_path_headings(forward_points)
        
        for i, (point, heading) in enumerate(zip(forward_points, forward_headings)):
            all_points.append((point[0], point[1], heading))
            all_reverse_flags.append(False)
        
        # Step 2: 转向点（车辆停止并转向）
        # 添加转向点，车头方向转为倒车朝向
        turn_point = reverse_start_pos
        turn_heading = reverse_heading_target  # 倒车时车头朝向
        all_points.append((turn_point[0], turn_point[1], turn_heading))
        all_reverse_flags.append(False)  # 转向时不移动
        
        # Step 3: 倒车段
        # 从转向点倒车到途径点
        backing_points = self.create_straight_backing_path(reverse_start_pos, end_pos, 8)
        
        for point in backing_points[1:]:  # 跳过第一个点（已添加）
            all_points.append((point[0], point[1], turn_heading))  # 倒车时车头方向不变
            all_reverse_flags.append(True)
        
        return ReversePath(all_points, all_reverse_flags)
    
    def generate_forward_path(self, start_pos, start_heading, end_pos, end_heading, control_point):
        """生成正常前进路径"""
        forward_points = self.create_smooth_path(start_pos, start_heading, end_pos, end_heading, control_point, 20)
        forward_headings = self.calculate_path_headings(forward_points)
        
        points_with_headings = [(p[0], p[1], h) for p, h in zip(forward_points, forward_headings)]
        reverse_flags = [False] * len(points_with_headings)
        
        return ReversePath(points_with_headings, reverse_flags)
    
    def create_smooth_path(self, start_pos, start_heading, end_pos, end_heading, control_point, num_points):
        """创建光滑路径（基于方向约束的贝塞尔曲线）"""
        # 根据起始和结束方向创建控制点
        control_distance = np.linalg.norm(np.array(end_pos) - np.array(start_pos)) * 0.3
        
        # 起始控制点：沿起始方向延伸
        ctrl1_x = start_pos[0] + control_distance * np.cos(start_heading)
        ctrl1_y = start_pos[1] + control_distance * np.sin(start_heading)
        
        # 结束控制点：沿结束方向反向延伸
        ctrl2_x = end_pos[0] - control_distance * np.cos(end_heading)
        ctrl2_y = end_pos[1] - control_distance * np.sin(end_heading)
        
        # 使用遗传算法控制点进行微调（确保control_point是2D点）
        if len(control_point) >= 2:
            ctrl1_x += (control_point[0] - start_pos[0]) * 0.3
            ctrl1_y += (control_point[1] - start_pos[1]) * 0.3
        
        # 生成三次贝塞尔曲线
        return self.cubic_bezier_with_controls(start_pos, (ctrl1_x, ctrl1_y), (ctrl2_x, ctrl2_y), end_pos, num_points)
    
    def create_straight_backing_path(self, start_pos, end_pos, num_points):
        """创建直线倒车路径"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y = start_pos[1] + t * (end_pos[1] - start_pos[1])
            points.append((x, y))
        return points
    
    def calculate_path_headings(self, points):
        """计算路径各点的车头朝向"""
        headings = []
        
        for i in range(len(points)):
            if i == 0:
                # 第一个点：使用与下一个点的方向
                if len(points) > 1:
                    dx = points[1][0] - points[0][0]
                    dy = points[1][1] - points[0][1]
                    headings.append(np.arctan2(dy, dx))
                else:
                    headings.append(0.0)
            elif i == len(points) - 1:
                # 最后一个点：使用与前一个点的方向
                dx = points[-1][0] - points[-2][0]
                dy = points[-1][1] - points[-2][1]
                headings.append(np.arctan2(dy, dx))
            else:
                # 中间点：使用前后点的平均方向
                dx = points[i+1][0] - points[i-1][0]
                dy = points[i+1][1] - points[i-1][1]
                headings.append(np.arctan2(dy, dx))
        
        return headings
    
    def cubic_bezier_with_controls(self, p0, p1, p2, p3, num_points):
        """生成四点控制的三次贝塞尔曲线"""
        t = np.linspace(0, 1, num_points)
        points = []
        for i in t:
            x = (1-i)**3 * p0[0] + 3*(1-i)**2*i * p1[0] + 3*(1-i)*i**2 * p2[0] + i**3 * p3[0]
            y = (1-i)**3 * p0[1] + 3*(1-i)**2*i * p1[1] + 3*(1-i)*i**2 * p2[1] + i**3 * p3[1]
            points.append((x, y))
        return points
    
    def cubic_bezier(self, p0, p1, p2, num_points):
        """生成三次贝塞尔曲线点（保持向后兼容）"""
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