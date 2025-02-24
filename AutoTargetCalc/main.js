const fs = require("fs");
const ph = require('path');
// 输入数据

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

// try wirit to the pathplan path file.
const PP_PATH = "../src/main/deploy/pathplanner/paths/"

let testPath = "ap6_left.path";

const PATH_MODIFY_PROTO = {
    waypointRelativePos: 0.6,
    maxV: 3.5,
    maxA: 3.5,
}

function eventMarkerParser(obj) {
    if (!obj["eventMarkers"] || obj["eventMarkers"].length == 0) {
        obj["eventMarkers"] = [{
            "name": "LN",
            "waypointRelativePos": 0.75,
            "endWaypointRelativePos": null,
            "command": null
        }];
    }

    return obj;
}
function eventMarkerLNTime(obj, newT) {
    for (let i = 0; i < obj["eventMarkers"].length; ++i) {
        let o = obj["eventMarkers"][i];
        if (o.name == "LN") {
            o.waypointRelativePos = newT;
        }
    }
    return obj;
}
function targetPos(obj, x, y) {
    let wp = obj["waypoints"];
    if (wp.length != 2) {
        throw "waypoint error!";
    }
    let firstP = wp[0];
    let lastP = wp[1];
    firstP.nextControl.x = x;
    firstP.nextControl.y = y;

    lastP.anchor.x = x;
    lastP.anchor.y = y;
    return obj;
}

function speed(obj, v, a) {
    let constrain = obj['globalConstraints'];
    constrain.maxVelocity = v;
    constrain.maxAcceleration = a;

    obj['goalEndState'].velocity = 0;
    obj['idealStartingState'].velocity = v;

    obj['useDefaultConstraints'] = false;
}
function processPathPlannerPath(aprilTagName, isLeft, newTargetX, newTargetY) {
    let filename = ph.join(PP_PATH, aprilTagName + "_" + (isLeft ? "left" : "right") + ".path");
    console.log(filename)
    // read filename as a json text file
    let text = fs.readFileSync(filename);
    let data = JSON.parse(text);
    targetPos(data, newTargetX, newTargetY)
    eventMarkerParser(data);
    eventMarkerLNTime(data, PATH_MODIFY_PROTO.waypointRelativePos);
    speed(data, PATH_MODIFY_PROTO.maxV, PATH_MODIFY_PROTO.maxA);
    fs.writeFileSync(filename, JSON.stringify(data, null, 4), 'utf8');
}


function main() {
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

        processPathPlannerPath(key, true, r.left.x, r.left.y);
        processPathPlannerPath(key, false, r.right.x, r.right.y);
    }

    return result;
}


main();