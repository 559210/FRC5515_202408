// 输入数据

// const ap17_pos = { x: 4.073906, y: 3.306318, theta: 330 };
// const ap18_pos = { x: 3.6576, y: 4.0259, theta: 270 };
// const ap19_pos = { x: 4.073906, y: 4.745482, theta: 210 };
// const ap20_pos = { x: 4.90474, y: 4.7475482, theta: 150 };
// const ap21_pos = { x: 5.321046, y: 4.0259, theta: 90 };
// const ap22_pos = { x: 4.90474, y: 3.306318, theta: 30 };
// const ap6_pos = { x: 13.474446, y: 3.306318, theta: 30 };
// const ap7_pos = { x: 13.890498, y: 4.0259, theta: 90 };
// const ap8_pos = { x: 13.474446, y: 4.745482, theta: 150 };
// const ap9_pos = { x: 12.643358, y: 4.745482, theta: 210 };
// const ap10_pos = { x: 12.227306, y: 4.0259, theta: 270 };
// const ap11_pos = { x: 12.643358, y: 3.306318, theta: 330 };

const APRIL_TAG = {
    "ap17": { x: 4.073906, y: 3.306318, theta: 330 },
    "ap18": { x: 3.6576, y: 4.0259, theta: 270 },
    "ap19": { x: 4.073906, y: 4.745482, theta: 210 },
    "ap20": { x: 4.90474, y: 4.7475482, theta: 150 },
    "ap21": { x: 5.321046, y: 4.0259, theta: 90 },
    "ap22": { x: 4.90474, y: 3.306318, theta: 30 },
    "ap6": { x: 13.474446, y: 3.306318, theta: 30 },
    "ap7": { x: 13.890498, y: 4.0259, theta: 90 },
    "ap8": { x: 13.474446, y: 4.745482, theta: 150 },
    "ap9": { x: 12.643358, y: 4.745482, theta: 210 },
    "ap10": { x: 12.227306, y: 4.0259, theta: 270 },
    "ap11": { x: 12.643358, y: 3.306318, theta: 330 },
}





// const midPoints = [
//     ap22_pos, ap21_pos, ap20_pos, ap19_pos, ap18_pos, ap17_pos,
//     // ap6_pos, ap7_pos, ap8_pos, ap9_pos, ap10_pos, ap11_pos,
// ];

// const angles = [30, 90, 150, 210, 270, 330]; // 每条边相对于 X 轴正方向的角度（单位：度）

const N = -.51; // 垂直偏移量
const OFFSET = 0.12; // 沿边偏移量, >0 right, < 0 left


function calcOffsetPoint(midPoint, angle, n, offset) {
        const angleRad = (angle * Math.PI) / 180; // 将角度转换为弧度

        // 计算垂直偏移点
        const perpendicularAngleRad = angleRad + Math.PI / 2; // 垂直方向的角度（弧度）
        const px = midPoint.x + n * Math.cos(perpendicularAngleRad); // 垂直偏移点的 x 坐标
        const py = midPoint.y + n * Math.sin(perpendicularAngleRad); // 垂直偏移点的 y 坐标

        // 计算沿边偏移后的新点
        const newPx = px + offset * Math.cos(angleRad); // 新点的 x 坐标
        const newPy = py + offset * Math.sin(angleRad); // 新点的 y 坐标

        return {
            midPoint: midPoint, // 原始中点
            perpendicularPoint: { x: px, y: py }, // 垂直偏移点
            newPoint: { x: newPx, y: newPy } // 沿边偏移后的新点
        };
}
// 计算垂直偏移点和沿边偏移后的新点
// function calculateAllOffsetPoints(midPoints, angles, n, offset) {
//     return midPoints.map((midPoint, index) => {
//         const angle = angles[index]; // 当前边的角度
//         const angleRad = (angle * Math.PI) / 180; // 将角度转换为弧度

//         // 计算垂直偏移点
//         const perpendicularAngleRad = angleRad + Math.PI / 2; // 垂直方向的角度（弧度）
//         const px = midPoint.x + n * Math.cos(perpendicularAngleRad); // 垂直偏移点的 x 坐标
//         const py = midPoint.y + n * Math.sin(perpendicularAngleRad); // 垂直偏移点的 y 坐标

//         // 计算沿边偏移后的新点
//         const newPx = px + offset * Math.cos(angleRad); // 新点的 x 坐标
//         const newPy = py + offset * Math.sin(angleRad); // 新点的 y 坐标

//         return {
//             midPoint: midPoint, // 原始中点
//             perpendicularPoint: { x: px, y: py }, // 垂直偏移点
//             newPoint: { x: newPx, y: newPy } // 沿边偏移后的新点
//         };
//     });
// }


function calcAllOffsetPoints() {
    let result = {};
    for (let key in APRIL_TAG) {
        let apInfo = APRIL_TAG[key];

        let r = {
            left: null,
            right: null,
        }
        
        r.right = calcOffsetPoint({x : apInfo.x, y: apInfo.y}, apInfo.theta, N, OFFSET).newPoint;
        r.left = calcOffsetPoint({x : apInfo.x, y: apInfo.y}, apInfo.theta, N, -OFFSET).newPoint;
        result[key] = r;
    }

    return result;
}

// // 计算结果
// const result = calculateOffsetPoints(midPoints, angles, n, offset);

// // 输出结果
// console.log("计算结果：");
// result.forEach((res, index) => {
//     console.log(`边 ${index + 1}:`);
//     console.log(`  中点坐标: (${res.midPoint.x}, ${res.midPoint.y})`);
//     console.log(`  垂直偏移点坐标: (${res.perpendicularPoint.x.toFixed(2)}, ${res.perpendicularPoint.y.toFixed(2)})`);
//     console.log(`  沿边偏移后的新点坐标: (${res.newPoint.x.toFixed(2)}, ${res.newPoint.y.toFixed(2)})`);
//     console.log("-----------------------------");
// });

const result = calcAllOffsetPoints();

console.log(result);