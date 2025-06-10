import numpy as np
from shapely.geometry import Polygon, LineString, Point
from config import *

def calculate_fitness(network):
    """计算路网个体的适应度值（倒车版本）"""
    total_fitness = 0
    
    # 提取所有路径段
    segments = [seg.line for seg in network.path_segments]
    
    # 1. 计算路径交叉数量
    crossings = calculate_crossings(segments)
    total_fitness += WEIGHTS['crossing'] * crossings
    
    # 2. 计算总路径长度
    total_length = sum(segment.length for segment in segments)
    total_fitness += WEIGHTS['length'] * total_length
    
    # 3. 边界违规检测
    boundary_violation = calculate_boundary_violation(segments)
    total_fitness += WEIGHTS['boundary'] * boundary_violation
    
    # 4. 路径重叠检测
    overlap_penalty = calculate_overlap_penalty(segments)
    total_fitness += WEIGHTS['overlap'] * overlap_penalty
    
    # 5. 曲率约束
    curvature_violation = calculate_curvature_violation(network.path_segments)
    total_fitness += WEIGHTS['curvature'] * curvature_violation
    
    # 6. 倒车操作惩罚
    reverse_count, reverse_length = calculate_reverse_metrics(network.path_segments)
    total_fitness += WEIGHTS['reverse_count'] * reverse_count
    total_fitness += WEIGHTS['reverse_length'] * reverse_length
    
    return total_fitness

def calculate_crossings(segments):
    """计算路径段之间的交叉点数"""
    crossings = 0
    for i in range(len(segments)):
        for j in range(i+1, len(segments)):
            if segments[i].crosses(segments[j]):
                crossings += 1
    return crossings

def calculate_boundary_violation(segments):
    """计算路径超出边界的程度"""
    poly = Polygon(BOUNDARY)
    violation = 0
    for segment in segments:
        if not poly.contains(segment):
            diff = segment.difference(poly)
            if not diff.is_empty:
                violation += diff.length
    return violation

def calculate_overlap_penalty(segments):
    """计算路径之间过近的惩罚项"""
    penalty = 0
    for i in range(len(segments)):
        for j in range(i+1, len(segments)):
            if i != j:
                distance = segments[i].distance(segments[j])
                if distance < SAFE_DISTANCE:
                    penalty += (SAFE_DISTANCE - distance) * 10
    return penalty

def calculate_curvature_violation(segments):
    """计算曲率违规（主要针对倒车段）"""
    violation = 0
    for segment in segments:
        points = segment.points
        reverse_flags = segment.reverse_flags
        
        for i in range(1, len(points)-1):
            # 计算三点之间的曲率
            p0 = np.array(points[i-1][:2])
            p1 = np.array(points[i][:2])
            p2 = np.array(points[i+1][:2])
            
            # 向量计算
            v1 = p0 - p1
            v2 = p2 - p1
            
            # 计算转弯半径
            chord_length = np.linalg.norm(p2 - p0)
            cross_product = abs(v1[0]*v2[1] - v1[1]*v2[0])  # Fixed: added multiplication operator
            
            if cross_product < 1e-6:
                continue
                
            radius = chord_length / (2 * cross_product / (np.linalg.norm(v1) * np.linalg.norm(v2)))  # Fixed: added multiplication operators
            
            # 确定允许的最小转弯半径
            min_radius = MIN_TURNING_RADIUS
            if reverse_flags[i]:
                min_radius *= REVERSE_TURNING_FACTOR
                
            # 添加违规惩罚
            if radius < min_radius:
                violation += min_radius - radius
                
    return violation

def calculate_reverse_metrics(segments):
    """计算倒车相关指标"""
    reverse_count = 0
    reverse_length = 0
    
    for segment in segments:
        reverse_count += segment.reverse_count
        reverse_length += segment.reverse_length
    
    # 应用最大倒车长度限制
    reverse_length = max(0, reverse_length - MAX_REVERSE_LENGTH * len(segments))
    
    return reverse_count, reverse_length