import numpy as np
from shapely.geometry import Point, LineString, Polygon
import math

def point_to_line_distance(point, line_start, line_end):
    """计算点到线段的距离"""
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end
    
    # 线段长度
    line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    if line_length == 0:
        return np.sqrt((x0 - x1)**2 + (y0 - y1)**2)
    
    # 计算投影参数
    t = max(0, min(1, ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / line_length**2))
    
    # 投影点
    proj_x = x1 + t * (x2 - x1)
    proj_y = y1 + t * (y2 - y1)
    
    return np.sqrt((x0 - proj_x)**2 + (y0 - proj_y)**2)

def line_intersection(line1_start, line1_end, line2_start, line2_end):
    """计算两条线段的交点"""
    x1, y1 = line1_start
    x2, y2 = line1_end
    x3, y3 = line2_start
    x4, y4 = line2_end
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    
    if abs(denom) < 1e-10:
        return None  # 平行线
    
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
    
    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection_x = x1 + t * (x2 - x1)
        intersection_y = y1 + t * (y2 - y1)
        return (intersection_x, intersection_y)
    
    return None

def calculate_angle(p1, p2, p3):
    """计算三点形成的角度（以p2为顶点）"""
    v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
    v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
    
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    
    return np.arccos(cos_angle)

def rotate_point(point, center, angle):
    """绕中心点旋转点"""
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    
    dx = point[0] - center[0]
    dy = point[1] - center[1]
    
    new_x = center[0] + dx * cos_a - dy * sin_a
    new_y = center[1] + dx * sin_a + dy * cos_a
    
    return (new_x, new_y)

def polygon_area(vertices):
    """使用鞋带公式计算多边形面积"""
    n = len(vertices)
    area = 0.0
    
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i][0] * vertices[j][1]
        area -= vertices[j][0] * vertices[i][1]
    
    return abs(area) / 2.0

def point_in_polygon(point, polygon_vertices):
    """射线法判断点是否在多边形内"""
    x, y = point
    n = len(polygon_vertices)
    inside = False
    
    p1x, p1y = polygon_vertices[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon_vertices[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    
    return inside

def calculate_turning_radius(p1, p2, p3):
    """计算三点间的转弯半径"""
    # 使用三点圆公式
    a = np.linalg.norm(np.array(p2) - np.array(p1))
    b = np.linalg.norm(np.array(p3) - np.array(p2))
    c = np.linalg.norm(np.array(p3) - np.array(p1))
    
    if a * b * c == 0:
        return float('inf')
    
    # 使用海伦公式计算面积
    s = (a + b + c) / 2
    area = np.sqrt(max(0, s * (s - a) * (s - b) * (s - c)))
    
    if area == 0:
        return float('inf')
    
    # 外接圆半径
    radius = (a * b * c) / (4 * area)
    return radius

def normalize_angle(angle):
    """将角度标准化到[-π, π]范围"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

def bezier_curve(control_points, num_points=100):
    """生成贝塞尔曲线"""
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_points)
    curve_points = []
    
    for t in t_values:
        point = [0, 0]
        for i in range(n + 1):
            binomial_coeff = math.comb(n, i)
            point[0] += binomial_coeff * (t ** i) * ((1 - t) ** (n - i)) * control_points[i][0]
            point[1] += binomial_coeff * (t ** i) * ((1 - t) ** (n - i)) * control_points[i][1]
        curve_points.append(tuple(point))
    
    return curve_points