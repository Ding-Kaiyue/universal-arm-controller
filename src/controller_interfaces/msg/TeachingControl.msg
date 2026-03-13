# 示教控制命令消息
# 支持暂停、恢复、取消、完成等操作

string mapping              # 指定要操作的映射 ("single_arm", "left_arm", "right_arm", "*")
string action               # 操作类型: "Start" (开始录制), "Pause" (暂停), "Resume" (恢复), "Cancel" (取消), "Complete" (完成)
string filename             # 文件名前缀（在所有操作中指定要操作的录制文件）