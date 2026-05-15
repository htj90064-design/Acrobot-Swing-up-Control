# Acrobot Swing-up and Balance Control
## 双摆系统的能量成型与 LQR 切换控制

本项目针对典型的欠驱动机器人系统——起重机双摆（Acrobot），设计并实现了从底部悬垂状态到顶部倒立平衡状态的全过程控制与动作仿真。

### 🌟 项目特性 / Features
- **严谨的动力学建模**：基于拉格朗日方程建立了 Acrobot 欠驱动系统高精度的非线性动力学数学模型。
- **混合双模切换控制 (Dual-mode Control)**：
  - **起摆阶段 (Swing-up)**：利用能量成型（Energy Shaping）与偏反馈线性化将能量平滑泵入系统，使其逼近顶部极值点状态。
  - **镇定阶段 (Balance)**：当系统状态判定进入局部吸引域（ROA）界限后，无缝切换为 LQR（线性二次型调节器）实现强鲁棒性的定点倒立平衡镇定。
- **高帧率物理直觉可视化**：使用底层的数值积分器演算，提供了 50 FPS 电影级极其丝滑连续的动态回放 GIF，以及非常详尽的角度/角速度/转矩/系统总能量等维度的时序分析曲线。

### 🛠️ 依赖与环境 / Dependencies
本项目基于 Python 实现仿真运算。建议使用 Python 3.8 及以上版本。
需安装以下基础的科学计算与绘图依赖包：
```bash
pip install numpy scipy matplotlib
```

### 🚀 运行与使用 / Usage
在终端中直接运行主程序脚本，即可自动进行后台物理仿真循环，并在完成后生成所有的状态图表与物理动画：
```bash
python generate_plots.py
```
单次执行完毕后，主目录下将成功导出多张分析截图（`.png`）以及名为 `fig5_4_acrobot_animation.gif` 的最终渲染动画展示文件。

### 📂 项目目录结构 / Repository Structure
```text
.
├── generate_plots.py            # 核心执行脚本：集成动力学解算、LQR镇定控制器、迭代函数及绘图渲染逻辑
├── 江鹤檀-2024312419-Project1.md # 随附的完整版课程设计报告（Markdown 版，包含详尽的理论与公式推导）
├── README.md                    # 本项目 GitHub 说明文档
└── 运行后生成的文件素材 / Generated Resources
    ├── fig5_1_state_response.png        # 关节角度与角速度响应比较图
    ├── fig5_2_control_input.png         # 控制器计算得出的带有硬阀值限幅的输出力矩变化图
    ├── fig5_3B_phase_plane.png          # 双关节的状态相平面相图
    ├── fig5_3_energy.png                # Acrobot 系统的整体能量逼近曲线图
    └── fig5_4_acrobot_animation.gif     # 生成得到的极其直观的物理引擎连杆起摆平衡动图
```

### 🎥 视觉演示 / Demo Showcase
您可以直接点击并查看根目录下生成的 **`fig5_4_acrobot_animation.gif`** 进行非常直观的过程验证，也可查阅课程设计报告 `.md` 深入理解系统各项约束实验表现。

### 🧑‍🎓 开发者信息 / Author
- **姓名**：jht
- 本项目为专项课程设计 Project 1 所对应的全部工程代码库分支。
