import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Polygon, LineString
import numpy as np

def plot_roadnet(network, boundary, start_point, end_point, waypoints):
    """可视化路网（含倒车指示和方向变化）"""
    plt.figure(figsize=(12, 12))
    ax = plt.gca()
    
    # 绘制区域边界
    poly = Polygon(boundary)
    x, y = poly.exterior.xy
    plt.plot(x, y, 'k-', linewidth=2)
    
    # 绘制入口点和出口点与朝向
    plt.plot(start_point[0], start_point[1], 'go', markersize=12, label='Start')
    plt.plot(end_point[0], end_point[1], 'ro', markersize=12, label='End')
    
    # 绘制起点朝向箭头
    from config import START_HEADING, END_HEADING
    start_arrow_length = 0.8
    start_dx = start_arrow_length * np.cos(START_HEADING)
    start_dy = start_arrow_length * np.sin(START_HEADING)
    plt.arrow(start_point[0], start_point[1], start_dx, start_dy, 
              head_width=0.2, head_length=0.2, fc='green', ec='green')
    
    # 绘制终点朝向箭头
    end_dx = start_arrow_length * np.cos(END_HEADING)
    end_dy = start_arrow_length * np.sin(END_HEADING)
    plt.arrow(end_point[0], end_point[1], end_dx, end_dy,
              head_width=0.2, head_length=0.2, fc='red', ec='red')
    
    # 绘制途径点和朝向
    for i, pt in enumerate(waypoints):
        plt.plot(pt[0], pt[1], 'bo', markersize=10)
        plt.text(pt[0] + 0.3, pt[1] + 0.3, f'W{i}', fontsize=12, ha='center', va='bottom')
        
        # 绘制途径点朝向（向内）
        heading = pt[2]  # waypoint的第三个元素是朝向
        arrow_length = 0.6
        dx = arrow_length * np.cos(heading)
        dy = arrow_length * np.sin(heading)
        plt.arrow(pt[0], pt[1], dx, dy, 
                  head_width=0.15, head_length=0.15, fc='blue', ec='blue', alpha=0.7)
    
    # 绘制路径
    colors = plt.cm.tab10.colors
    arrow_style = patches.ArrowStyle("->", head_length=4, head_width=2)
    
    for i, segment in enumerate(network.path_segments):
        color = colors[i % len(colors)]
        
        # 绘制路径线
        if isinstance(segment.line, LineString):
            x, y = segment.line.xy
            
            # 分别绘制前进段和倒车段
            points = segment.points
            reverse_flags = segment.reverse_flags
            
            # 绘制前进段（实线）
            forward_x, forward_y = [], []
            reverse_x, reverse_y = [], []
            
            for j in range(len(points)):
                if not reverse_flags[j]:
                    forward_x.append(points[j][0])
                    forward_y.append(points[j][1])
                else:
                    reverse_x.append(points[j][0])
                    reverse_y.append(points[j][1])
            
            # 绘制前进段
            if len(forward_x) > 1:
                plt.plot(forward_x, forward_y, color=color, linewidth=2.5, alpha=0.8, label=f'Path {i+1} Forward')
            
            # 绘制倒车段（虚线）
            if len(reverse_x) > 1:
                plt.plot(reverse_x, reverse_y, color=color, linewidth=2.5, alpha=0.8, 
                        linestyle='--', label=f'Path {i+1} Reverse')
            
            # 标记转向点（前进到倒车的转换点）
            for j in range(1, len(points)):
                if not reverse_flags[j-1] and reverse_flags[j]:
                    # 这是从前进到倒车的转换点
                    plt.plot(points[j-1][0], points[j-1][1], 'ks', markersize=8, alpha=0.8)
                    plt.text(points[j-1][0], points[j-1][1] + 0.2, 'TURN', 
                            fontsize=8, ha='center', va='bottom', weight='bold')
            
            # 添加车辆朝向箭头（每隔几个点）
            for j in range(0, len(points), max(1, len(points)//5)):  # 显示5个朝向箭头
                if j < len(points):
                    px, py, heading = points[j]
                    arrow_length = 0.3
                    dx = arrow_length * np.cos(heading)
                    dy = arrow_length * np.sin(heading)
                    
                    # 前进段用实心箭头，倒车段用空心箭头
                    if reverse_flags[j]:
                        # 倒车段：空心红色箭头
                        plt.arrow(px, py, dx, dy, head_width=0.1, head_length=0.1, 
                                 fc='none', ec='red', linewidth=1.5)
                    else:
                        # 前进段：实心蓝色箭头
                        plt.arrow(px, py, dx, dy, head_width=0.1, head_length=0.1, 
                                 fc='blue', ec='blue', alpha=0.6)
    
    # 添加图例元素
    plt.plot([], [], 'b-', linewidth=2.5, alpha=0.8, label='Forward Path')
    plt.plot([], [], 'b--', linewidth=2.5, alpha=0.8, label='Reverse Path')
    plt.plot([], [], 'ks', markersize=8, alpha=0.8, label='Turn Point')
    plt.arrow(0, 0, 0, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue', alpha=0.6, label='Vehicle Heading (Forward)')
    plt.arrow(0, 0, 0, 0, head_width=0.1, head_length=0.1, fc='none', ec='red', linewidth=1.5, label='Vehicle Heading (Reverse)')
    
    plt.legend(loc='upper left', bbox_to_anchor=(1.02, 1))
    
    plt.title("Road Network with Realistic Backing Maneuvers\n(Showing Sharp Turns and Direction Changes)")
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