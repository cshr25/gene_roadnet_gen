import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Polygon, LineString
import numpy as np

def plot_roadnet(network, boundary, start_point, end_point, waypoints):
    """可视化路网（含倒车指示）"""
    plt.figure(figsize=(12, 12))
    ax = plt.gca()
    
    # 绘制区域边界
    poly = Polygon(boundary)
    x, y = poly.exterior.xy
    plt.plot(x, y, 'k-', linewidth=2)
    
    # 绘制入口点和出口点
    plt.plot(start_point[0], start_point[1], 'go', markersize=12, label='Start')
    plt.plot(end_point[0], end_point[1], 'ro', markersize=12, label='End')
    
    # 绘制途径点
    for i, pt in enumerate(waypoints):
        plt.plot(pt[0], pt[1], 'bo', markersize=10)
        plt.text(pt[0], pt[1], f'W{i}', fontsize=12, ha='center', va='bottom')
    
    # 绘制路径
    colors = plt.cm.tab10.colors
    arrow_style = patches.ArrowStyle("->", head_length=6, head_width=3)
    
    for i, segment in enumerate(network.path_segments):
        color = colors[i % len(colors)]
        
        # 绘制路径线
        if isinstance(segment.line, LineString):
            x, y = segment.line.xy
            plt.plot(x, y, color=color, linewidth=1.5, alpha=0.7)
            
            # 在倒车段添加特殊标记
            points = segment.points
            reverse_flags = segment.reverse_flags
            
            for j in range(len(points)-1):
                if reverse_flags[j]:
                    mid_x = (points[j][0] + points[j+1][0]) / 2
                    mid_y = (points[j][1] + points[j+1][1]) / 2
                    plt.plot(mid_x, mid_y, 'rx', markersize=8)
                    
            # 添加倒车方向指示箭头
            if len(points) > 1 and any(reverse_flags):
                # 查找第一个倒车段
                start_idx = None
                end_idx = None
                for j in range(len(points)-1):
                    if reverse_flags[j]:
                        start_idx = j
                        while j < len(points)-1 and reverse_flags[j]:
                            j += 1  # Fixed: proper increment
                        end_idx = j
                        break
                
                if start_idx is not None and end_idx is not None and end_idx - start_idx > 1:
                    arrow_idx = (start_idx + end_idx) // 2
                    if arrow_idx < len(points) - 1:
                        dx = points[arrow_idx+1][0] - points[arrow_idx][0]
                        dy = points[arrow_idx+1][1] - points[arrow_idx][1]
                        arrow = patches.FancyArrowPatch(
                            (points[arrow_idx][0], points[arrow_idx][1]),
                            (points[arrow_idx][0] + dx*0.5, points[arrow_idx][1] + dy*0.5),  # Fixed: added multiplication operator
                            arrowstyle=arrow_style,
                            color='red',
                            mutation_scale=15
                        )
                        ax.add_patch(arrow)
    
    # 添加图例
    plt.plot([], [], 'rx', markersize=8, label='Reverse Segment')
    plt.legend(loc='best')
    
    plt.title("Road Network with Reverse Maneuvers")
    plt.axis('equal')
    plt.grid(True)
    plt.savefig('road_network_reverse.png', dpi=300)
    plt.show()

def save_network_data(network):
    """保存网络数据到文件"""
    with open('network_data.txt', 'w') as f:
        f.write("Road Network Analysis Report\n")
        f.write("=" * 40 + "\n\n")
        
        # 基本统计
        total_reverse_count = 0
        total_reverse_length = 0
        total_path_length = 0
        
        f.write("Path Segment Details:\n")
        f.write("-" * 20 + "\n")
        
        for i, segment in enumerate(network.path_segments):
            total_reverse_count += segment.reverse_count
            total_reverse_length += segment.reverse_length
            total_path_length += segment.line.length
            
            f.write(f"Segment {i+1}:\n")
            f.write(f"  Length: {segment.line.length:.2f} meters\n")
            f.write(f"  Reverse segments: {segment.reverse_count}\n")
            f.write(f"  Reverse length: {segment.reverse_length:.2f} meters\n")
            f.write(f"  Points: {len(segment.points)}\n\n")
        
        f.write("Summary Statistics:\n")
        f.write("-" * 20 + "\n")
        f.write(f"Total path length: {total_path_length:.2f} meters\n")
        f.write(f"Total reverse segments: {total_reverse_count}\n")
        f.write(f"Total reverse length: {total_reverse_length:.2f} meters\n")
        f.write(f"Reverse ratio: {(total_reverse_length/total_path_length)*100:.1f}%\n")

def plot_fitness_evolution(fitness_history):
    """绘制适应度进化曲线"""
    plt.figure(figsize=(10, 6))
    plt.plot(fitness_history, 'b-', linewidth=2)
    plt.title('Fitness Evolution Over Generations')
    plt.xlabel('Generation')
    plt.ylabel('Best Fitness')
    plt.grid(True, alpha=0.3)
    plt.savefig('fitness_evolution.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_path_analysis(network):
    """绘制路径分析图"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # 1. 路径长度分析
    segment_lengths = [seg.line.length for seg in network.path_segments]
    ax1.bar(range(len(segment_lengths)), segment_lengths, color='skyblue')
    ax1.set_title('Path Segment Lengths')
    ax1.set_xlabel('Segment Index')
    ax1.set_ylabel('Length (meters)')
    ax1.grid(True, alpha=0.3)
    
    # 2. 倒车段分析
    reverse_counts = [seg.reverse_count for seg in network.path_segments]
    reverse_lengths = [seg.reverse_length for seg in network.path_segments]
    
    x = range(len(reverse_counts))
    ax2.bar(x, reverse_counts, color='orange', alpha=0.7, label='Count')
    ax2_twin = ax2.twinx()
    ax2_twin.bar([i+0.4 for i in x], reverse_lengths, color='red', alpha=0.7, width=0.4, label='Length')
    
    ax2.set_title('Reverse Segment Analysis')
    ax2.set_xlabel('Segment Index')
    ax2.set_ylabel('Reverse Count', color='orange')
    ax2_twin.set_ylabel('Reverse Length (meters)', color='red')
    ax2.grid(True, alpha=0.3)
    
    # 3. 曲率分析（简化版）
    curvatures = []
    for segment in network.path_segments:
        points = segment.points
        segment_curvatures = []
        for i in range(1, len(points)-1):
            # 简化曲率计算
            p0 = np.array(points[i-1][:2])
            p1 = np.array(points[i][:2])
            p2 = np.array(points[i+1][:2])
            
            v1 = p1 - p0
            v2 = p2 - p1
            
            # 角度变化作为曲率指标
            angle_change = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))
            segment_curvatures.append(angle_change)
        
        if segment_curvatures:
            curvatures.append(np.mean(segment_curvatures))
        else:
            curvatures.append(0)
    
    ax3.plot(curvatures, 'g-o', linewidth=2, markersize=6)
    ax3.set_title('Average Curvature per Segment')
    ax3.set_xlabel('Segment Index')
    ax3.set_ylabel('Average Curvature (radians)')
    ax3.grid(True, alpha=0.3)
    
    # 4. 路径效率分析
    total_lengths = []
    direct_distances = []
    efficiency_ratios = []
    
    from config import START_POINT, END_POINT, WAYPOINTS
    
    # 计算每个途径点的路径效率
    for i, waypoint in enumerate(WAYPOINTS):
        # 入口到途径点
        segment_to = network.path_segments[i]
        direct_dist_to = np.linalg.norm(np.array(waypoint) - np.array(START_POINT))
        efficiency_to = direct_dist_to / segment_to.line.length if segment_to.line.length > 0 else 0
        
        # 途径点到出口
        segment_from = network.path_segments[i + len(WAYPOINTS)]
        direct_dist_from = np.linalg.norm(np.array(END_POINT) - np.array(waypoint))
        efficiency_from = direct_dist_from / segment_from.line.length if segment_from.line.length > 0 else 0
        
        efficiency_ratios.extend([efficiency_to, efficiency_from])
    
    ax4.bar(range(len(efficiency_ratios)), efficiency_ratios, color='purple', alpha=0.7)
    ax4.set_title('Path Efficiency (Direct Distance / Actual Path)')
    ax4.set_xlabel('Path Index')
    ax4.set_ylabel('Efficiency Ratio')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=1.0, color='red', linestyle='--', alpha=0.5, label='Perfect Efficiency')
    ax4.legend()
    
    plt.tight_layout()
    plt.savefig('path_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()