## CMakeLists.txt 文件修复计划

### 问题分析

当前的 CMakeLists.txt 文件存在以下问题：

1. **缺少 cr7\_test\_main.cpp 的构建配置**：我们已经修复了 cr7\_test\_main.cpp 文件，但是 CMakeLists.txt 中没有为它创建可执行文件
2. **依赖项可能不完整**：需要确保所有必要的依赖项都被正确声明
3. **构建结构优化**：可以改进构建结构，使代码更清晰

### 修复方案

#### 1. 添加 cr7\_test\_main 可执行文件

* 创建新的可执行文件 `cr7_test_main`

* 包含 `test/cr7_test_main.cpp` 源文件

* 链接必要的依赖项

#### 2. 优化构建结构

* 将共享的源文件提取到变量中

* 为主可执行文件和测试可执行文件分别配置源文件

* 确保依赖项的一致性

#### 3. 确保依赖项完整

* 检查并添加所有必要的依赖项

* 确保 tf2\_geometry\_msgs 被正确链接

* 验证所有包含目录都正确设置

### 预期结果

修复后，CMakeLists.txt 文件将：

* 正确构建主可执行文件 `cr7_controller`

* 正确构建测试可执行文件 `cr7_controller_test`

