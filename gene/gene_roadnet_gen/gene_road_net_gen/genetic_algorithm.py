import random
import numpy as np
from shapely.geometry import Polygon, Point
from road_network import RoadNetwork
from constraints import calculate_fitness
from config import *

class GeneticAlgorithm:
    def __init__(self):
        self.best_solution = None
        self.best_fitness = float('inf')
        self.population = self.initialize_population()
        self.evaluate_population()  # Initialize best solution
    
    def initialize_population(self):
        """初始化种群"""
        return [RoadNetwork() for _ in range(POPULATION_SIZE)]
    
    def evaluate_population(self):
        """评估整个种群的适应度"""
        for network in self.population:
            fitness = calculate_fitness(network)
            if fitness < self.best_fitness:
                self.best_fitness = fitness
                self.best_solution = network
    
    def select_parents(self):
        """锦标赛选择父代"""
        tournament = random.sample(self.population, TOURNAMENT_SIZE)
        return min(tournament, key=lambda net: calculate_fitness(net))
    
    def crossover(self, parent1, parent2):
        """
        混合交叉 - 处理新的6参数基因结构
        每个途径点：[ctrl1_x, ctrl1_y, turn_distance, turn_angle, ctrl2_x, ctrl2_y]
        """
        child_genes = []
        
        # 按途径点进行交叉，保持参数组的完整性
        for waypoint_idx in range(len(WAYPOINTS)):
            # 随机选择从哪个父代继承这个途径点的参数
            if random.random() < 0.5:
                parent = parent1
            else:
                parent = parent2
            
            # 复制该途径点的所有6个参数
            gene_start = waypoint_idx * 6
            for param_idx in range(6):
                child_genes.append(parent.genes[gene_start + param_idx])
                
        return RoadNetwork(genes=child_genes)
    
    def mutate(self, network):
        """
        变异操作 - 处理新的6参数基因结构
        每个途径点：[ctrl1_x, ctrl1_y, turn_distance, turn_angle, ctrl2_x, ctrl2_y]
        """
        new_genes = network.genes.copy()
        
        # 按途径点进行变异
        for waypoint_idx in range(len(WAYPOINTS)):
            gene_start = waypoint_idx * 6
            
            # 变异控制点1 (x, y)
            if random.random() < MUT_PROBABILITY:
                new_genes[gene_start] += random.uniform(-1.0, 1.0)      # ctrl1_x
                new_genes[gene_start + 1] += random.uniform(-1.0, 1.0)  # ctrl1_y
            
            # 变异转向距离
            if random.random() < MUT_PROBABILITY:
                delta_distance = random.uniform(-0.3, 0.3)
                new_distance = new_genes[gene_start + 2] + delta_distance
                new_genes[gene_start + 2] = np.clip(new_distance, TURN_DISTANCE_MIN, TURN_DISTANCE_MAX)
            
            # 变异转向角度
            if random.random() < MUT_PROBABILITY:
                delta_angle = random.uniform(-np.pi/12, np.pi/12)  # ±15度
                new_angle = new_genes[gene_start + 3] + delta_angle
                new_genes[gene_start + 3] = np.clip(new_angle, -TURN_ANGLE_MAX, TURN_ANGLE_MAX)
            
            # 变异控制点2 (x, y)
            if random.random() < MUT_PROBABILITY:
                new_genes[gene_start + 4] += random.uniform(-1.0, 1.0)  # ctrl2_x
                new_genes[gene_start + 5] += random.uniform(-1.0, 1.0)  # ctrl2_y
        
        # 有概率进行倒车优化变异
        if random.random() < MUT_PROBABILITY * 0.5:  # 降低频率
            self.optimize_reverse_segments(new_genes)
            
        return RoadNetwork(genes=new_genes)
    
    def optimize_reverse_segments(self, genes):
        """
        专门优化倒车段的变异操作 - 新基因结构
        每个途径点：[ctrl1_x, ctrl1_y, turn_distance, turn_angle, ctrl2_x, ctrl2_y]
        """
        # 随机选择一个途径点进行优化
        waypoint_idx = random.randint(0, len(WAYPOINTS) - 1)
        waypoint = WAYPOINTS[waypoint_idx]
        waypoint_pos = (waypoint[0], waypoint[1])
        
        gene_start = waypoint_idx * 6
        
        # 优化策略1：减少转向距离，使倒车段更短
        if random.random() < 0.5:
            current_distance = genes[gene_start + 2]
            new_distance = current_distance * 0.8  # 减少20%
            genes[gene_start + 2] = max(new_distance, TURN_DISTANCE_MIN)
        
        # 优化策略2：调整转向角度，使其更接近垂直进入
        if random.random() < 0.5:
            genes[gene_start + 3] *= 0.7  # 减少角度偏移
        
        # 优化策略3：调整控制点1，使前进路径更直接
        if random.random() < 0.3:
            ctrl1_x, ctrl1_y = genes[gene_start], genes[gene_start + 1]
            
            # 将控制点向途径点方向移动
            direction = np.array(waypoint_pos) - np.array([ctrl1_x, ctrl1_y])
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction) * 0.5
                new_ctrl = (ctrl1_x + direction[0], ctrl1_y + direction[1])
                
                # 确保在区域内
                poly = Polygon(BOUNDARY)
                if poly.contains(Point(new_ctrl)):
                    genes[gene_start] = new_ctrl[0]
                    genes[gene_start + 1] = new_ctrl[1]
            
    def evolve(self):
        """执行一代进化"""
        # 评估当前种群
        self.evaluate_population()
        
        # 创建新种群
        new_population = [self.best_solution]  # 精英保留
        
        while len(new_population) < POPULATION_SIZE:
            # 选择
            parent1 = self.select_parents()
            parent2 = self.select_parents()
            
            # 交叉
            if random.random() < CX_PROBABILITY:
                child = self.crossover(parent1, parent2)
            else:
                child = random.choice([parent1, parent2])
            
            # 变异
            child = self.mutate(child)
            
            new_population.append(child)
        
        self.population = new_population
    
    def run(self):
        """运行进化算法"""
        for gen in range(MAX_GENERATIONS):
            self.evolve()
            print(f"Generation {gen+1}: Best Fitness = {self.best_fitness}")
        
        return self.best_solution