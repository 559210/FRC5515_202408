// 输入数据
const M = { x: 3.6576, y: 4.0259 }; // 边的中点坐标
const theta = 270; // 边相对于 X 轴正方向的角度（单位：度）
const P = { x: 3.15, y: 3.91  }; // 正六边形外的点坐标


// 将角度转换为弧度
const thetaRad = (theta * Math.PI) / 180;

// 计算点 P 到边的垂直距离 n
const n = Math.abs(
    (P.x - M.x) * Math.sin(thetaRad) - (P.y - M.y) * Math.cos(thetaRad)
);

// 计算投影点 Q 的坐标
const Qx = P.x - n * Math.sin(thetaRad);
const Qy = P.y + n * Math.cos(thetaRad);

// 计算投影点 Q 到中点 M 的距离 offset
const offset = Math.sqrt((Qx - M.x) ** 2 + (Qy - M.y) ** 2);

// 判断 offset 的正负
const directionVector = { x: Math.cos(thetaRad), y: Math.sin(thetaRad) }; // 边的方向向量
const dotProduct = (Qx - M.x) * directionVector.x + (Qy - M.y) * directionVector.y;
const offsetSigned = dotProduct >= 0 ? offset : -offset;

// 输出结果
console.log("点到边的垂直距离 n:", n.toFixed(2));
console.log("投影点到中点的距离 offset:", offsetSigned.toFixed(2));
console.log("投影点 Q 的坐标:", { x: Qx.toFixed(2), y: Qy.toFixed(2) });