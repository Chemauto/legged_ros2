## Sim2Sim / Sim2Real 提示

- 关节顺序必须一致
- 检查 MuJoCo 模型里 joint 的 `armature`
- 检查 IMU 的安装位置和朝向
- 检查关节限位  
  如果 RL 策略在训练中利用了某些关节限位，而真机关节限位不同，策略效果通常会明显下降
