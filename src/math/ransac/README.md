**NOTE:**

​	这里内嵌 3d 平面、2d 圆的 ransac 算法结构。`ransac.cc` 是 ransac 算法的核心框架。利用该框架提供的接口编写了 `ransac_app.cc` 应用程序。仿照该程序可以编写其他模型的 ransac 应用。相应头文件可以在 `include/math/ransac` 中查看。在工程顶层目录的 `examples/ransac` 文件夹中包含了两个例子，利用了 `ransac_app.cc` 文件提供的接口。