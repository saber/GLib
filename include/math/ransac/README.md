ransac.h:       ransac 算法框架声明
ransac_app.h: 完成的基于 ransac 框架的应用，包括 3d 平面拟合、2d 圆拟合。支持同一数据点的多模型拟合。提供两种模型拟合接口函数，供外部使用。

ransac_models.h: 定义用于 ransac 拟合的模型集。包括 T3Dplane、Circle 两个模型。分别代表 3d 平面、2d 圆。

***note***: 可以添加其他模型，相应接口可在 ransac_app.h ransac_models.h 文件中编写。