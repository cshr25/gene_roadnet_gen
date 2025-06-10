import numpy as np
from shapely.geometry import Point, LineString
from config import *

def generate_reeds_shepp_path(start, end, is_reverse):
    """
    生成Reeds-Shepp路径（支持前进和倒车）
    Enhanced implementation with proper curve handling
    """
    # 计算起点和终点之间的距离和方向
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = np.sqrt(dx**2 + dy**2)
    
    if distance < 0.1:  # 距离太近，直接返回直线
        return LineString([start, end])
    
    # 计算路径方向
    angle = np.arctan2(dy, dx)
    
    # 生成路径点
    num_points = max(10, int(distance * 3))
    points = []
    
    for i in range(num_points):
        t = i / (num_points - 1)
        
        # 添加轻微的S形曲线以模拟车辆动力学
        curve_factor = 0.3 * np.sin(t * np.pi) if is_reverse else 0.1 * np.sin(t * np.pi)
        
        # 垂直于主方向的偏移
        offset_x = -curve_factor * np.sin(angle)
        offset_y = curve_factor * np.cos(angle)
        
        x = start[0] + t * dx + offset_x
        y = start[1] + t * dy + offset_y
        points.append((x, y))
    
    return LineString(points)

def optimize_with_bezier(points, reverse_segment=False):
    """使用贝塞尔曲线优化路径"""
    if len(points) < 4:
        return points
    
    optimized_points = []
    
    # 分段优化，每4个点形成一个贝塞尔曲线
    for i in range(0, len(points) - 3, 3):
        p0 = np.array(points[i][:2])
        p1 = np.array(points[i+1][:2])
        p2 = np.array(points[i+2][:2])
        p3 = np.array(points[i+3][:2])
        
        # 生成贝塞尔曲线点
        bezier_points = generate_cubic_bezier(p0, p1, p2, p3, 10)
        
        # 调整倒车段的曲率
        if reverse_segment:
            # 倒车时使用更大的转弯半径
            bezier_points = reduce_curvature(bezier_points, factor=REVERSE_TURNING_FACTOR)
        
        optimized_points.extend(bezier_points[:-1])  # 避免重复点
    
    # 添加最后一个点
    optimized_points.append(points[-1][:2])
    
    return optimized_points

def validate_reverse_constraints(path, reverse_flags):
    """验证倒车约束"""
    if not path or len(path) < 3:
        return True
    
    violations = 0
    
    for i in range(1, len(path) - 1):
        if reverse_flags[i]:
            # 计算当前点的曲率半径
            p0 = np.array(path[i-1][:2])
            p1 = np.array(path[i][:2])
            p2 = np.array(path[i+1][:2])
            
            # 计算转弯半径
            radius = calculate_turning_radius(p0, p1, p2)
            
            # 检查是否满足倒车转弯半径要求
            min_reverse_radius = MIN_TURNING_RADIUS * REVERSE_TURNING_FACTOR
            if radius < min_reverse_radius:
                violations += 1
    
    # 检查连续倒车长度
    continuous_reverse_length = 0
    max_continuous_reverse = 0
    
    for i in range(len(path) - 1):
        if reverse_flags[i]:
            p1 = np.array(path[i][:2])
            p2 = np.array(path[i+1][:2])
            continuous_reverse_length += np.linalg.norm(p2 - p1)
        else:
            max_continuous_reverse = max(max_continuous_reverse, continuous_reverse_length)
            continuous_reverse_length = 0
    
    # 检查最大连续倒车长度
    max_continuous_reverse = max(max_continuous_reverse, continuous_reverse_length)
    if max_continuous_reverse > MAX_REVERSE_LENGTH:
        violations += 1
    
    return violations == 0

def calculate_path_curvature(points):
    """计算路径的曲率"""
    curvatures = []
    for i in range(1, len(points) - 1):
        p0 = np.array(points[i-1])
        p1 = np.array(points[i])
        p2 = np.array(points[i+1])
        
        # 计算曲率使用三点圆公式
        a = np.linalg.norm(p1 - p0)
        b = np.linalg.norm(p2 - p1)
        c = np.linalg.norm(p2 - p0)
        
        if a * b * c == 0:
            curvatures.append(0)
            continue
            
        # 使用海伦公式计算面积
        s = (a + b + c) / 2
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        
        # 曲率 = 4 * area / (a * b * c)
        curvature = 4 * area / (a * b * c)
        curvatures.append(curvature)
    
    return curvatures

def smooth_path(points, window_size=3):
    """平滑路径点"""
    if len(points) < window_size:
        return points
    
    smoothed = []
    for i in range(len(points)):
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(points), i + window_size // 2 + 1)
        
        avg_x = np.mean([p[0] for p in points[start_idx:end_idx]])
        avg_y = np.mean([p[1] for p in points[start_idx:end_idx]])
        
        if len(points[i]) > 2:  # 包含航向角
            smoothed.append((avg_x, avg_y, points[i][2]))
        else:
            smoothed.append((avg_x, avg_y))
    
    return smoothed

def generate_dubins_path(start, end, radius):
    """生成简化的Dubins路径"""
    # 简化实现：直接连接起点和终点
    # 实际实现应使用Dubins几何
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = np.sqrt(dx**2 + dy**2)
    
    num_points = max(10, int(distance * 5))
    points = []
    
    for i in range(num_points):
        t = i / (num_points - 1)
        x = start[0] + t * dx
        y = start[1] + t * dy
        points.append((x, y))
    
    return points

def generate_cubic_bezier(p0, p1, p2, p3, num_points):
    """生成三次贝塞尔曲线点"""
    t = np.linspace(0, 1, num_points)
    points = []
    
    for i in t:
        # 三次贝塞尔曲线公式
        x = (1-i)**3 * p0[0] + 3*(1-i)**2*i * p1[0] + 3*(1-i)*i**2 * p2[0] + i**3 * p3[0]
        y = (1-i)**3 * p0[1] + 3*(1-i)**2*i * p1[1] + 3*(1-i)*i**2 * p2[1] + i**3 * p3[1]
        points.append((x, y))
    
    return points

def reduce_curvature(points, factor=1.5):
    """降低路径曲率"""
    if len(points) < 3:
        return points
    
    reduced_points = []
    for i in range(len(points)):
        if i == 0 or i == len(points) - 1:
            reduced_points.append(points[i])
        else:
            # 计算平滑点
            prev_p = np.array(points[i-1])
            curr_p = np.array(points[i])
            next_p = np.array(points[i+1])
            
            # 计算向量
            v1 = curr_p - prev_p
            v2 = next_p - curr_p
            
            # 计算平滑后的点
            smooth_p = curr_p + (v1 + v2) / (2 * factor)
            reduced_points.append(tuple(smooth_p))
    
    return reduced_points

def calculate_turning_radius(p0, p1, p2):
    """计算三点间的转弯半径"""
    # 计算向量
    v1 = p1 - p0
    v2 = p2 - p1
    
    # 计算夹角
    dot_product = np.dot(v1, v2)
    norms = np.linalg.norm(v1) * np.linalg.norm(v2)
    
    if norms < 1e-10:
        return float('inf')
    
    cos_angle = np.clip(dot_product / norms, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    
    if angle < 1e-6:  # 直线
        return float('inf')
    
    # 计算转弯半径
    chord_length = np.linalg.norm(p2 - p0)
    radius = chord_length / (2 * np.sin(angle / 2))
    
    return radius