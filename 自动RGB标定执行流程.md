# OpenART 自动 RGB 标定执行流程

本文档配套脚本：[rgb_auto_calibrate.py](rgb_auto_calibrate.py)。目标是在现场自动测出车辆和地图元素的 RGB 色值，然后把结果填回正式识别脚本 [main copy.py](main%20copy.py)。

## 1. 适用范围

当前方案不区分车头车尾，视觉只需要稳定输出车辆所在格子。车辆仍需要测两个颜色原型：

- `CAR_HEAD_RGB`：实际可当作车辆绿色原型。
- `CAR_TAIL_RGB`：实际可当作车辆青色原型。

这两个名字沿用当前代码，含义不再强调车头/车尾。正式脚本最后仍会输出单一 `@` 和 `(car_x, car_y)`。

## 2. 标定前准备

1. 固定 OpenART 摄像头位置。
2. 固定屏幕位置和屏幕亮度。
3. 保持比赛或调试时使用的环境光，不要中途大幅改变补光。
4. 确认正式脚本里已经关闭自动增益和自动白平衡：
   - `sensor.set_auto_gain(False)`
   - `sensor.set_auto_whitebal(False)`
5. 先完成四角几何标定，确保采样点落在格子中心。

## 3. 第一步：同步四角网格参数

打开 [main copy.py](main%20copy.py)，把正式识别脚本中已经调好的参数复制到 [rgb_auto_calibrate.py](rgb_auto_calibrate.py)：

```python
GRID_CORNERS = {
    "tl": (..., ...),
    "tr": (..., ...),
    "bl": (..., ...),
    "br": (..., ...),
}

GRID_K1 = ...
```

如果四角还没有调准，请先不要测 RGB。因为采样格子不准时，自动测出来的颜色会混入格线或相邻元素，后面阈值会越调越乱。

## 4. 第二步：准备一张标定画面

让屏幕上的 16x12 地图中尽量同时包含以下元素：

- 车辆绿色半块
- 车辆青色半块
- 暗墙、亮墙
- 暗空地、亮空地
- 暗目的地、亮目的地
- 暗箱子、亮箱子
- 暗炸弹、亮炸弹

如果某类元素当前画面没有，可以先保留脚本里的格子坐标，后续再单独开一张包含该元素的画面重测。

## 5. 第三步：填写采样格子坐标

打开 [rgb_auto_calibrate.py](rgb_auto_calibrate.py)，找到 `SAMPLE_SPECS`。

每一行格式是：

```python
("变量名", 格子x, 格子y, 像素偏移x, 像素偏移y)
```

坐标规则：

- 左上角格子是 `(0, 0)`。
- 右下角格子是 `(15, 11)`。
- `格子x` 向右增加。
- `格子y` 向下增加。
- 像素偏移以该格中心为原点，向右为正，向下为正。

普通元素建议偏移填 `0, 0`：

```python
("WALL_DARK_RGB", 0, 0, 0, 0)
```

车辆双色块如果在同一个格子里，需要分别采绿色半块和青色半块。比如车辆在 `(7, 5)`，绿色在左、青色在右，可以写：

```python
("CAR_HEAD_RGB", 7, 5, -4, 0)
("CAR_TAIL_RGB", 7, 5,  4, 0)
```

如果预览里采样十字没有落到对应半块中心，就调整最后两个偏移值。偏移不必很大，通常 `-6` 到 `+6` 像素内就够。

## 6. 第四步：运行自动标定脚本

1. 将 [rgb_auto_calibrate.py](rgb_auto_calibrate.py) 复制到 OpenART SD 卡根目录。
2. 在 SD 卡根目录把它改名为 `main.py`。
3. 插回 OpenART，重启或运行脚本。
4. 打开 OpenMV IDE 串口终端观察输出。
5. 程序启动后会等待 `START_DELAY_MS`，默认 3 秒。
6. 之后连续采样 `CALIB_FRAMES` 帧，默认 50 帧。
7. 采样结束后会在终端打印所有 RGB 结果。
8. 同时会在 SD 卡根目录生成 `rgb_calibration.txt`。

输出示例：

```python
CAR_HEAD_RGB = (25, 182, 0)
CAR_TAIL_RGB = (0, 182, 239)
WALL_DARK_RGB = (78, 96, 118)
WALL_BRIGHT_RGB = (132, 150, 172)
```

## 7. 第五步：把结果填回正式脚本

打开正式识别脚本 [main copy.py](main%20copy.py)，把 `rgb_calibration.txt` 里的结果填回这些变量：

```python
CAR_HEAD_RGB = (..., ..., ...)
CAR_TAIL_RGB = (..., ..., ...)
WALL_DARK_RGB = (..., ..., ...)
WALL_BRIGHT_RGB = (..., ..., ...)
FLOOR_DARK_RGB = (..., ..., ...)
FLOOR_BRIGHT_RGB = (..., ..., ...)
GOAL_DARK_RGB = (..., ..., ...)
GOAL_BRIGHT_RGB = (..., ..., ...)
BOX_DARK_RGB = (..., ..., ...)
BOX_BRIGHT_RGB = (..., ..., ...)
BOMB_DARK_RGB = (..., ..., ...)
BOMB_BRIGHT_RGB = (..., ..., ...)
```

如果正式运行版本在 SD 卡根目录 [main.py](../main.py)，还需要把同样的参数同步过去。

## 8. 第六步：验证颜色结果

把正式识别脚本重新部署到 OpenART 后，观察终端 ASCII 地图：

1. 静止画面下，地图字符应基本稳定，不应大面积闪烁。
2. 车辆所在格应稳定输出为 `@`。
3. `car_x, car_y` 应与屏幕上虚拟车格子一致。
4. 如果车辆漏检，优先略微增大 `SYMBOL_MAX_DIST_SQ["H"]` 和 `SYMBOL_MAX_DIST_SQ["T"]`。
5. 如果其他元素被误判为车，优先减小车辆阈值，或重新检查车辆 RGB 是否采到了格线/混色区域。

## 9. 常见问题

### 9.1 测出来的 RGB 明显不对

优先检查：

1. `GRID_CORNERS` 是否和正式脚本一致。
2. `SAMPLE_SPECS` 的格子坐标是否填错。
3. 采样十字是否落在目标颜色中心。
4. 是否采到了格线、阴影、反光或相邻元素。

### 9.2 车辆绿色和青色测反了

不影响“不区分车头车尾”的定位思路。当前正式脚本仍用 `H/T` 两个入口，最终输出单一 `@`。只要两个颜色都能稳定识别到车辆即可。

### 9.3 有些元素当前地图没有

可以先只测有的元素。没有出现的元素保留旧值，等切换到包含该元素的地图后，再改 `SAMPLE_SPECS` 单独重测。

### 9.4 光照变化后识别又不稳

RGB 标定和屏幕亮度、环境光强相关。比赛前应在最终光照条件下重新跑一遍 [rgb_auto_calibrate.py](rgb_auto_calibrate.py)。

## 10. 现场执行清单

```text
[ ] 摄像头固定
[ ] 屏幕位置固定
[ ] 屏幕亮度固定
[ ] 自动增益关闭
[ ] 自动白平衡关闭
[ ] 四角 GRID_CORNERS 已调准
[ ] GRID_CORNERS / GRID_K1 已同步到 rgb_auto_calibrate.py
[ ] SAMPLE_SPECS 已填写每类元素所在格
[ ] 车辆绿/青采样偏移已对准半块中心
[ ] 脚本已复制到 SD 卡根目录并命名为 main.py
[ ] 已运行并生成 rgb_calibration.txt
[ ] RGB 结果已填回正式 main copy.py
[ ] 正式脚本已重新部署
[ ] ASCII 地图稳定
[ ] car_x / car_y 与虚拟车位置一致
```
