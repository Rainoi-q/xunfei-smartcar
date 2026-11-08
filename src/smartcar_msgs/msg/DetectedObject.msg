# 单个检测目标（货物、投递点等）
string label         # 类别名，如 纸箱 / 包裹 / 投递点
float32 confidence   # 置信度 0.0-1.0

# 像素坐标边界框（左上角、右下角）
int32 x_min
int32 y_min
int32 x_max
int32 y_max
