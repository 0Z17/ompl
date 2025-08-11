# 约束规划系统 - 完整重构版本

这是一个完全重构的约束规划系统，支持多种规划算法，包括环境约束规划的具体实现。

## 功能特性

### 支持的规划算法
- **FMT**: Fast Marching Tree
- **PCSFMT**: Point Cloud Sampling FMT
- **AtlasRRTstar**: 基于Atlas的约束RRT*算法
- **BundleBITstar**: 基于Bundle的约束BIT*算法

### 约束规划功能
- 表面约束规划
- 约束状态空间定义
- 自动图表锚定
- 约束状态有效性检查
- 约束路径后处理

### 系统组件
- **NURBS表面重建**: 基于点云数据的表面拟合
- **逆运动学求解**: 5自由度机械臂运动学
- **碰撞检测**: 基于MuJoCo的物理仿真
- **路径优化**: 多种优化目标和权重配置
- **可视化渲染**: 实时路径和表面渲染

## 文件结构

```
├── PcsFmtTest.cpp          # 主要的规划系统实现
├── test_planning.cpp       # 测试程序
├── config.yaml            # 配置文件
├── CMakeLists.txt         # 编译配置
├── README.md              # 说明文档
├── ConstrainedPlanningCommon.h  # 约束规划通用定义
└── PlanningConfig.h       # 配置管理类
```

## 编译说明

### 依赖项
- OMPL (Open Motion Planning Library)
- Eigen3
- PCL (Point Cloud Library)
- yaml-cpp
- GLFW3
- MuJoCo
- 自定义库: dynamic_planning, surface_reconstructor, mujoco_client

### 编译步骤

```bash
# 创建构建目录
mkdir build && cd build

# 配置CMake
cmake ..

# 编译
make

# 运行测试
./test_planning
```

## 配置说明

配置文件 `config.yaml` 包含以下主要部分：

### 文件路径配置
```yaml
files:
  pcd_file: "点云文件路径"
  model_file: "MuJoCo模型文件路径"
  output_dir: "输出目录"
```

### 规划参数
```yaml
planning:
  type: "AtlasRRTstar"  # 规划算法类型
  sample_num: 5000      # 采样数量
  planning_timeout: 20.0 # 规划超时时间
  atlas_timeout: 5.0    # Atlas超时时间
```

### 状态空间边界
```yaml
state_bounds:
  x: [0.0, 3.0]
  y: [-3.0, 3.0]
  z: [0.0, 3.5]
  psi: [-1.5708, 1.5708]
  theta: [-1.5708, 1.5708]
```

### 约束参数
```yaml
surface_constraint:
  tolerance: 0.003  # 约束容差
```

## 使用方法

### 基本使用

```cpp
#include "PcsFmtTest.cpp"

int main() {
    // 创建规划系统
    PlanningSystem planner("config.yaml");
    
    // 初始化系统
    if (!planner.initialize()) {
        return -1;
    }
    
    // 运行规划
    planner.runPlanning();
    
    return 0;
}
```

### 约束规划特性

1. **表面约束**: 系统自动从点云数据构建NURBS表面，并定义表面约束
2. **约束状态空间**: 使用OMPL的ConstrainedStateSpace处理约束
3. **图表锚定**: 对于AtlasRRTstar，自动进行图表锚定
4. **约束验证**: 专门的状态有效性检查器处理约束状态

### 输出结果

- **路径文件**: CSV格式的规划路径
- **规划数据**: 包含规划统计信息
- **表面文件**: STL格式的重建表面（可选）
- **可视化**: 实时渲染的规划结果

## 算法说明

### AtlasRRTstar
- 基于Atlas流形的约束RRT*算法
- 适用于复杂表面约束规划
- 自动图表锚定和切换

### BundleBITstar
- 基于Bundle的约束BIT*算法
- 渐近最优的约束规划
- 高效的批量采样策略

### FMT/PCSFMT
- 无约束的快速规划算法
- 适用于自由空间规划
- 支持点云采样优化

## 故障排除

### 常见问题

1. **编译错误**: 检查依赖项是否正确安装
2. **配置错误**: 验证config.yaml中的文件路径
3. **规划失败**: 调整规划参数和约束容差
4. **渲染问题**: 确保GLFW和OpenGL正确配置

### 调试建议

- 启用详细日志输出
- 检查状态空间边界设置
- 验证约束定义的正确性
- 调整规划超时时间

## 扩展开发

### 添加新的规划算法

1. 在`PlanningType`枚举中添加新类型
2. 在`planWithType`方法中添加处理逻辑
3. 实现相应的设置和求解方法

### 自定义约束

1. 继承`ob::Constraint`类
2. 实现`function`方法定义约束
3. 在`setupConstrainedPlanning`中使用新约束

### 配置扩展

1. 在`PlanningConfig.h`中添加新配置项
2. 更新`config.yaml`模板
3. 在相应的初始化代码中使用新配置

## 许可证

本项目遵循相关开源许可证。