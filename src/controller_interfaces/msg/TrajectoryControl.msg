# 轨迹控制命令消息
# 支持暂停、恢复、取消轨迹执行
# 注：一个mapping同时只能执行一个轨迹，所以不需要execution_id

string mapping              # 指定要操作的映射 ("single_arm", "left_arm", "right_arm")
string action              # 操作类型: "Pause" (暂停), "Resume" (恢复), "Cancel" (取消)
