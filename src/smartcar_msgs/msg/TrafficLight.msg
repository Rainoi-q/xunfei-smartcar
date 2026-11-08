# 红绿灯识别结果（对应功能⑩）
std_msgs/Header header

# 颜色常量
uint8 UNKNOWN=0
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=3

uint8 color          # 识别到的灯色，取值见上方常量
float32 confidence   # 置信度 0.0-1.0
uint32 area_px       # 目标区域像素面积，用于过滤远处误检
bool valid           # 本帧是否检测有效
