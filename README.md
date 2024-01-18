# 从仿真学习控制理论

本仓库仅提供相关仿真与代码，不对其原理进行解释，如需了解相关控制器的理论，请移步到我的[知乎](https://www.zhihu.com/people/hariRobotics)。仓库的更新频率与文章的更新频率不一致，如果发现两边有对不上的内容，请催更我。

## 系统需求与建议
* 本仓库所有Simulink仿真均基于MATLAB R2023b版本，提供同时提供R2019b版本备份，不保证完全兼容
* 本仓库可能有部分代码涉及非开源工具箱，会提供MATLAB以外的方法供参考

## 目录
* [LQR入门 - 通过使用MATLAB自带的lqr函数求解状态反馈矩阵](lqr/)
* [ISMC-LQR控制器 - 一种可以集成到其他控制器中的鲁棒控制器](ismc/)
* [MRAC - 在模型已知的情况下对外部缓慢变化的干扰自适应的控制器](mrac/)
* [再探LQR - 使用凸优化求解LQR控制问题](lqr_convex/)
* [Back-stepping - 反步控制与非线性系统入门](backstepping/)
* [ISMC-Back-stepping - 对非线性系统的鲁棒控制](ismc_backstepping_baseline/)

TODO:
* NDI - 非线性动态逆

## 如何进行仿真
1. 使用MATLAB打开对应控制器仿真文件夹
2. 运行`init_model.m`，将所有变量加载到工作区。
3. 运行仿真文件
4. python文件根据依赖项安装对应的包以后直接运行即可