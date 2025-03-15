const fs = require("fs");
const ph = require('path');
// 输入数据

const APRIL_TAG = {
    // algea 0: low, 1: high
    "ap17": { id: 17, x: 4.073906, y: 3.306318, theta: 330, algea: 0, sourceList:[12] },
    "ap18": { id: 18,  x: 3.6576, y: 4.0259, theta: 270, algea: 1, sourceList:[12, 13] },
    "ap19": { id: 19, x: 4.073906, y: 4.745482, theta: 210, algea: 0, sourceList:[13] },
    "ap20": { id: 20, x: 4.90474, y: 4.7475482, theta: 150, algea: 1, sourceList:[13] },
    "ap21": { id: 21, x: 5.321046, y: 4.0259, theta: 90, algea: 0, sourceList:[12, 13] },
    "ap22": { id: 22, x: 4.90474, y: 3.306318, theta: 30, algea: 1, sourceList:[12] },
    "ap6": { id: 6, x: 13.474446, y: 3.306318, theta: 30, algea: 0, sourceList:[1] },
    "ap7": { id: 7, x: 13.890498, y: 4.0259, theta: 90, algea: 1, sourceList:[1, 2] },
    "ap8": { id: 8, x: 13.474446, y: 4.745482, theta: 150, algea: 0, sourceList:[2] },
    "ap9": { id: 9, x: 12.643358, y: 4.745482, theta: 210, algea: 1, sourceList:[2] },
    "ap10": { id: 10, x: 12.227306, y: 4.0259, theta: 270, algea: 0, sourceList:[1, 2] },
    "ap11": { id: 11, x: 12.643358, y: 3.306318, theta: 330, algea: 1, sourceList:[1] },
}


const APRIL_TAG_POS = {
    LEFT: "left",
    CENTER: "center",
    RIGHT: "right",
}

const N = -.55; // 垂直偏移量
const OFFSET = 0.164338; // 沿边偏移量, >0 right, < 0 left

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
const PP_PATH = __dirname + "/../src/main/deploy/pathplanner/paths/"

let testPath = "ap6_left.path";

const PATH_MODIFY_PROTO = {
    waypointRelativePos: 1.6,
    maxV: 2,//3.5,
    maxA: 2,//3.5,
}

function calculateNewPoint(x, y, theta, d) {
    // 将角度转换为弧度
    const radians = theta * (Math.PI / 180);

    // 计算新点的坐标
    const newX = x + d * Math.cos(radians);
    const newY = y + d * Math.sin(radians);

    return { x: newX, y: newY };
}

function eventMarkerParser(obj, aprilTagName, aprilTagPos) {
    console.log("ddd: ", aprilTagName, aprilTagPos);
    if (aprilTagPos == APRIL_TAG_POS.CENTER) {
        // if (!obj["eventMarkers"] || obj["eventMarkers"].length == 0) {
            obj["eventMarkers"] = [{
                "name": APRIL_TAG[aprilTagName].algea == 0 ? "Ball1" : "Ball2",
                "waypointRelativePos": 0.75,
                "endWaypointRelativePos": null,
                "command": null
            }];
        // }

    }
    else {
        // if (!obj["eventMarkers"] || obj["eventMarkers"].length == 0) {
            obj["eventMarkers"] = [{
                "name": "LN",
                "waypointRelativePos": 0.75,
                "endWaypointRelativePos": null,
                "command": null
            }];
        // }
    }

    return obj;
}

function contsrainZoneParser(obj) {
    let zones = [];
    let count = obj["waypoints"].length - 1;
    let v = PATH_MODIFY_PROTO.maxV / count;
    let a = PATH_MODIFY_PROTO.maxA / count;
    for (let i = 0; i < count; ++i) {
        let zone = {
            "name": "Constraints Zone" + "_" + i,
            "minWaypointRelativePos": i,
            "maxWaypointRelativePos": i + 1,
            "constraints": {
                "maxVelocity": PATH_MODIFY_PROTO.maxV, // - v * i,
                "maxAcceleration": PATH_MODIFY_PROTO.maxA,// - a * i,
                "maxAngularVelocity": 540,
                "maxAngularAcceleration": 720,
                "nominalVoltage": 12,
                "unlimited": false
            }
        };
        zones.push(zone);
    }
    obj["constraintZones"] = zones;
}

function contsrainZoneParserBall(obj) {
    let zones = [];
    let count = obj["waypoints"].length - 1;
    let v = PATH_MODIFY_PROTO.maxV / count;
    let a = PATH_MODIFY_PROTO.maxA / count;
    for (let i = 0; i < count; ++i) {
        let zone = {
            "name": "Constraints Zone" + "_" + i,
            "minWaypointRelativePos": i,
            "maxWaypointRelativePos": i + 1,
            "constraints": {
                "maxVelocity": PATH_MODIFY_PROTO.maxV - v * i,
                "maxAcceleration": PATH_MODIFY_PROTO.maxA - a * i,
                "maxAngularVelocity": 540,
                "maxAngularAcceleration": 720,
                "nominalVoltage": 12,
                "unlimited": false
            }
        };
        zones.push(zone);
    }
    obj["constraintZones"] = zones;
}

function eventMarkerLNTime(obj, newT) {
    for (let i = 0; i < obj["eventMarkers"].length; ++i) {
        let o = obj["eventMarkers"][i];
        if (o.name == "LN" || o.name == "Ball1" || o.name == "Ball2") {
            o.waypointRelativePos = newT;
        }
    }
    return obj;
}

function targetPos(obj, x, y) {
    let wp = obj["waypoints"];
    if (wp.length < 2) {
        throw "waypoint error!";
    }
    // let firstP = wp[0];
    let lastP = wp[wp.length - 1];
    // firstP.nextControl.x = x;
    // firstP.nextControl.y = y;

    lastP.anchor.x = x;
    lastP.anchor.y = y;
    return obj;
}

function targetPosNew(obj, x, y, theta) {
    const count = 4;
    const wayPointOffset = -0.2;
    const controlPointOffset = 0.1;

    let points = new Array(count);
    for (let i = count - 1; i >= 0; --i) {
        let preControlPoint = calculateNewPoint(x, y, theta, -controlPointOffset);
        let nextControlPoint = calculateNewPoint(x, y, theta, controlPointOffset);
        let p = {
            "anchor": {
                "x": x,
                "y": y
            },
            "prevControl": (i == 0) ? null : preControlPoint,
            "nextControl": (i == count - 1) ? null : nextControlPoint,
            "isLocked": false,
            "linkedName": null
        }
        let nextPoint = calculateNewPoint(x, y, theta, wayPointOffset);
        x = nextPoint.x;
        y = nextPoint.y;

        points[i] = p;
    }

    obj["waypoints"] = points;

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
function processPathPlannerPath(aprilTagName, aprilTagPos, newTargetX, newTargetY, theta) {
    let filename = '';
    if (aprilTagPos == APRIL_TAG_POS.CENTER) {
        filename = ph.join(PP_PATH, aprilTagName + ".path");
        filename = filename.replace("ap", "ball");
    }
    else {
        filename = ph.join(PP_PATH, aprilTagName + "_" + aprilTagPos + ".path");
    }
    console.log(filename)
    // read filename as a json text file
    let text = '';
    if (fs.existsSync(filename)) {
        text = fs.readFileSync(filename);
    } else {
        text = fs.readFileSync(ph.join(PP_PATH, aprilTagName + "_" + APRIL_TAG_POS.LEFT + ".path"));
    }
    let data = JSON.parse(text);
    targetPosNew(data, newTargetX, newTargetY, theta + 90);
    eventMarkerParser(data, aprilTagName, aprilTagPos);
    eventMarkerLNTime(data, PATH_MODIFY_PROTO.waypointRelativePos);
    speed(data, PATH_MODIFY_PROTO.maxV, PATH_MODIFY_PROTO.maxA);
    if (aprilTagPos == APRIL_TAG_POS.CENTER) {
        contsrainZoneParserBall(data);
    }
    else {
        contsrainZoneParser(data);
    }
    
    fs.writeFileSync(filename, JSON.stringify(data, null, 4), 'utf8');
}

function processSourcePathPlannerPath(key, aprilTagPos, newTargetX, newTargetY, theta) {
    let apObj = APRIL_TAG[key];
    let sourceList = apObj.sourceList;
    for (let i = 0; i < sourceList.length; ++i) {
        let sourceId = sourceList[i];
        let sourcePathOrigin = ph.join(PP_PATH,  "source" + apObj.id + "-" + sourceId + ".path");
        let text = fs.readFileSync(sourcePathOrigin);

        let data = JSON.parse(text);
        let firstWp = data["waypoints"][0]['anchor'];
        firstWp.x = newTargetX;
        firstWp.y = newTargetY;

        data['idealStartingState'].velocity = 1;
        data["idealStartingState"].rotation = theta + 90;

        while (data["idealStartingState"].rotation > 360) {
            data["idealStartingState"].rotation -= 360;
        }

        while (data["idealStartingState"].rotation < 0) {
            data["idealStartingState"].rotation + 360;
        }

        let outFilename = ph.join(PP_PATH,  "source" + apObj.id + "-" + sourceId + "_" + aprilTagPos + ".path");
        fs.writeFileSync(outFilename, JSON.stringify(data, null, 4), 'utf8');
    }
    
}

function main() {
    let result = {};
    for (let key in APRIL_TAG) {
        let apInfo = APRIL_TAG[key];

        let r = {
            left: null,
            center: null,
            right: null,
        }
        
        r.right = calcOffsetPoint({x : apInfo.x, y: apInfo.y}, apInfo.theta, N, OFFSET).newPoint;
        r.left = calcOffsetPoint({x : apInfo.x, y: apInfo.y}, apInfo.theta, N, -OFFSET).newPoint;
        r.center = calcOffsetPoint({x : apInfo.x, y: apInfo.y}, apInfo.theta, N*1.2, 0).newPoint;
        result[key] = r;

        processPathPlannerPath(key, APRIL_TAG_POS.LEFT, r.left.x, r.left.y, apInfo.theta);
        processPathPlannerPath(key, APRIL_TAG_POS.CENTER, r.center.x, r.center.y, apInfo.theta);
        processPathPlannerPath(key, APRIL_TAG_POS.RIGHT, r.right.x, r.right.y, apInfo.theta);

        let np = calcOffsetPoint(r.left, apInfo.theta, -0.25, 0).newPoint
        processSourcePathPlannerPath(key, APRIL_TAG_POS.LEFT, np.x, np.y, apInfo.theta);
        np = calcOffsetPoint(r.right, apInfo.theta, -0.25, 0).newPoint
        processSourcePathPlannerPath(key, APRIL_TAG_POS.RIGHT, np.x, np.y, apInfo.theta);
    }

    return result;
}


main();