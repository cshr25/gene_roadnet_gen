from genetic_algorithm import GeneticAlgorithm
from visualization import plot_roadnet, plot_fitness_evolution, plot_path_analysis, save_network_data
from constraints import calculate_fitness
from config import *
import time

def main():
    try:
        print("Starting Reverse Road Network Optimization...")
        print(f"Configuration:")
        print(f"  Population size: {POPULATION_SIZE}")
        print(f"  Max generations: {MAX_GENERATIONS}")
        print(f"  Boundary: {BOUNDARY}")
        print(f"  Start point: {START_POINT}")
        print(f"  End point: {END_POINT}")
        print(f"  Waypoints: {WAYPOINTS}")
        print(f"  Min turning radius: {MIN_TURNING_RADIUS}")
        print(f"  Reverse turning factor: {REVERSE_TURNING_FACTOR}")
        print(f"  Max reverse length: {MAX_REVERSE_LENGTH}")
        
        # Validate configuration
        if POPULATION_SIZE <= 0 or MAX_GENERATIONS <= 0:
            raise ValueError("Population size and max generations must be positive")
        
        if len(WAYPOINTS) == 0:
            raise ValueError("At least one waypoint is required")
        
        start_time = time.time()
        
        # 初始化遗传算法
        print("\nInitializing genetic algorithm...")
        ga = GeneticAlgorithm()
        
        if ga.best_solution is None:
            raise RuntimeError("Failed to initialize population")
        
        # 记录适应度历史
        fitness_history = []
        
        # 运行进化过程
        print("\nRunning evolutionary optimization...")
        for gen in range(MAX_GENERATIONS):
            try:
                ga.evolve()
                fitness_history.append(ga.best_fitness)
                if (gen + 1) % 10 == 0:
                    print(f"Generation {gen+1}: Best Fitness = {ga.best_fitness:.4f}")
            except Exception as e:
                print(f"Warning: Error in generation {gen+1}: {e}")
                continue
        
        best_network = ga.best_solution
        
        if best_network is None:
            raise RuntimeError("Failed to find a valid solution")
        
        # 输出结果
        elapsed_time = time.time() - start_time
        print(f"\nOptimization completed in {elapsed_time:.2f} seconds")
        print(f"Best fitness: {ga.best_fitness:.4f}")
        
        # 计算详细统计信息
        total_reverse_count = 0
        total_reverse_length = 0
        total_path_length = 0
        
        for segment in best_network.path_segments:
            total_reverse_count += segment.reverse_count
            total_reverse_length += segment.reverse_length
            total_path_length += segment.line.length
        
        print(f"\nNetwork Statistics:")
        print(f"  Total path length: {total_path_length:.2f} meters")
        print(f"  Total reverse segments: {total_reverse_count}")
        print(f"  Total reverse length: {total_reverse_length:.2f} meters")
        print(f"  Average reverse length per waypoint: {total_reverse_length/len(WAYPOINTS):.2f} meters")
        print(f"  Reverse ratio: {(total_reverse_length/total_path_length)*100:.1f}%")
        
        # 保存网络数据
        save_network_data(best_network)
        print("\nNetwork data saved to 'network_data.txt'")
        
        # 可视化结果
        print("\nGenerating visualizations...")
        
        # 可视化最佳路网
        plot_roadnet(best_network, BOUNDARY, START_POINT, END_POINT, WAYPOINTS)
        
        # 绘制适应度进化曲线
        plot_fitness_evolution(fitness_history)
        
        # 绘制路径分析
        plot_path_analysis(best_network)
        
        print("All visualizations saved!")
        print("Files generated:")
        print("  - road_network_reverse.png")
        print("  - fitness_evolution.png") 
        print("  - path_analysis.png")
        print("  - network_data.txt")
        
    except KeyboardInterrupt:
        print("\nOptimization interrupted by user")
    except ValueError as e:
        print(f"Configuration error: {e}")
    except RuntimeError as e:
        print(f"Runtime error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()