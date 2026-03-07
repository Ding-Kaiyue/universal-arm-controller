# OpenClaw 扩展开发 Bug 总结

## 问题现象

用户在飞书与 OpenClaw 机器人对话时报错：
```
Cannot read properties of undefined (reading 'properties')
```

即使是简单的 "hi" 问候也会触发错误。三个工具（arm:movej:left, arm:movej:right, arm:health）虽然在启动时成功注册，但对话时仍然报错。

## 根本原因

**使用了错误的 OpenClaw 插件 API 格式**

### 错误的格式（我们最初的代码）

```javascript
api.registerTool({
  id: "arm:movej:left",              // ✗ 用了 id（应该是 name）
  name: "Move Left Arm",
  description: "...",
  inputSchema: {                      // ✗ 用了 inputSchema（应该是 parameters）
    type: "object",
    properties: { ... }
  },
  execute: async (input) => {         // ✗ 签名错误（应该是 (_toolCallId, params)）
    return executeArmCommand(...);
  }
});
```

### 正确的格式（官方 feishu 扩展）

```javascript
api.registerTool((ctx) => ({           // ✓ 接收一个函数，返回工具定义
  name: "arm_movej_left",             // ✓ 用 name（下划线风格）
  label: "Move Left Arm",             // ✓ 加上 label
  description: "...",
  parameters: {                        // ✓ 用 parameters
    type: "object",
    properties: { ... }
  },
  async execute(_toolCallId, params) { // ✓ 正确的签名
    return executeArmCommand(...);
  }
}));
```

## 为什么会报那个错误？

OpenClaw 框架在加载工具时的流程：

```
1. api.registerTool() 被调用
   ↓
2. OpenClaw 框架遍历所有工具定义
   ↓
3. 尝试访问 schema.properties
   ↓
4. 如果是错误的格式，这一步就会找不到 properties
   ↓
5. 报错：Cannot read properties of undefined (reading 'properties')
```

**关键点**：错误不是在注册时发生，而是在框架**处理**工具定义时发生。这就是为什么：
- 三个工具仍然能"成功注册"（console.log 打出来了）
- 但对话时报错（框架处理工具时失败了）

## 为什么这么难诊断？

1. **文档不清楚** - OpenClaw 官方文档中关于插件 API 的说明不够突出
2. **混淆的 API** - OpenClaw 插件 API 看起来像 Anthropic tool schema，但格式完全不同
3. **错误信息模糊** - `Cannot read properties of undefined (reading 'properties')` 没有指出具体是哪个工具的 schema 有问题
4. **没有 schema 验证** - OpenClaw 应该在工具注册时就验证格式，而不是等到后来使用时才报错

## 关键发现点

使用正确的方向：
- 查看官方 OpenClaw 仓库的参考实现（feishu 扩展）
- 对比官方代码和我们的代码，找出格式差异
- 发现了 4 个关键区别：
  1. `api.registerTool()` 的参数必须是函数，不是对象
  2. 字段名：`parameters` 而非 `inputSchema`
  3. 执行签名：`(_toolCallId, params)` 而非 `(input)`
  4. 字段名：`name` 而非 `id`

## 预防措施

### 对于开发者
1. 始终参考官方示例代码而非文档
2. 使用官方的 TypeScript 类型定义（如果有的话）
3. 模仿成功的扩展（如 feishu）的格式

### 对于 OpenClaw 框架维护者
1. 在工具注册时立即验证 schema 格式
2. 提供清晰的错误消息，指出具体哪个工具的哪个字段有问题
3. 优化文档，突出插件 API 的正确格式
4. 考虑提供一个 TypeScript 类型定义文件

## 最终结论

这不是 OpenClaw 框架的 bug，而是**我们使用了错误的 API 格式**。

根本原因：OpenClaw 的插件 API 文档不够清晰，导致容易误用。框架也没有在注册时进行足够的验证，而是等到后来使用时才报错。

**解决办法**：查看官方参考实现，改成正确的格式。
