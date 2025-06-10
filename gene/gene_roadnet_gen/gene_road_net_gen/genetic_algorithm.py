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
        """混合交叉"""
        child_genes = []
        
        # 随机选择交叉点
        crossover_point = random.randint(1, len(parent1.genes) - 1)
        
        # 从父代复制基因
        for i in range(len(parent1.genes)):
            if i < crossover_point:
                child_genes.append(parent1.genes[i])
            else:
                child_genes.append(parent2.genes[i])
                
        return RoadNetwork(genes=child_genes)
    
    def mutate(self, network):
        """变异操作（含倒车优化）"""
        new_genes = network.genes.copy()
        
        for i in range(len(new_genes)):
            if random.random() < MUT_PROBABILITY:
                # 轻微扰动控制点
                dx = random.uniform(-1.0, 1.0)
                dy = random.uniform(-1.0, 1.0)
                new_genes[i] = (new_genes[i][0] + dx, new_genes[i][1] + dy)
        
        # 有概率进行倒车优化变异
        if random.random() < MUT_PROBABILITY:
            self.optimize_reverse_segments(new_genes)
            
        return RoadNetwork(genes=new_genes)
    
    def optimize_reverse_segments(self, genes):
        """专门优化倒车段的变异操作"""
        # 随机选择一个途径点进行优化
        idx = random.randint(0, len(WAYPOINTS) - 1)
        
        # 调整控制点以减少倒车长度
        waypoint = WAYPOINTS[idx]
        
        # 进入途径点的路径控制点优化（使倒车段更短）
        ctrl_idx = idx * 2
        ctrl_point = list(genes[ctrl_idx])
        
        # 将控制点向途径点方向移动
        direction = np.array(waypoint) - np.array(ctrl_point)
        direction = direction / np.linalg.norm(direction) * 0.5
        new_ctrl = (
            ctrl_point[0] + direction[0],
            ctrl_point[1] + direction[1]
        )
        
        # 确保在区域内
        poly = Polygon(BOUNDARY)
        if poly.contains(Point(new_ctrl)):
            genes[ctrl_idx] = new_ctrl
            
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