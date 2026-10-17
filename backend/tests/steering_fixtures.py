"""Synthetic geometry for independent steering verification, never vehicle data."""

from copy import deepcopy


def design():
    return {
        "version": 1, "kind": "fixed-axis-steering", "name": "Synthetic test assembly",
        "coordinateSystem": "SAE_J670_Z_DOWN", "hardpointFrame": "AXLE_LOCAL", "lengthUnit": "mm",
        "wheelbase": 1600, "frontTrack": 1200, "rearTrack": 1180,
        "rackTravel": 120, "rackMmPerRevolution": -60, "steeringLimitDeg": 180,
        "left": {
            "axisUpper": [0, -550, -450], "axisLower": [0, -550, -150],
            "tieInner": [-100, -250, -250], "tieOuter": [-100, -520, -250],
            "wheelCenterZ": -250, "staticToeDeg": 0, "staticCamberDeg": 0,
        },
        "right": {
            "axisUpper": [0, 550, -450], "axisLower": [0, 550, -150],
            "tieInner": [-100, 250, -250], "tieOuter": [-100, 520, -250],
            "wheelCenterZ": -250, "staticToeDeg": 0, "staticCamberDeg": 0,
        },
    }


def tilted_design():
    result = design()
    result["left"].update(axisUpper=[30, -530, -450], axisLower=[-10, -560, -150],
                          staticToeDeg=0.4, staticCamberDeg=-2.2)
    result["right"].update(axisUpper=[22, 532, -440], axisLower=[-8, 563, -140],
                           tieInner=[-90, 245, -235], tieOuter=[-103, 519, -248],
                           staticToeDeg=0.7, staticCamberDeg=1.3)
    return result


def path_design(inner_x):
    result = design()
    result["rackMmPerRevolution"] = -360
    result["left"].update(axisUpper=[0, 0, -1], axisLower=[0, 0, 1],
                          tieOuter=[0, 2, 0], tieInner=[inner_x, 2, 0])
    return result


def scaled_design(source, factor):
    result = deepcopy(source)
    for key in ("wheelbase", "frontTrack", "rearTrack", "rackTravel", "rackMmPerRevolution"):
        result[key] *= factor
    for side in ("left", "right"):
        for key in ("axisUpper", "axisLower", "tieInner", "tieOuter"):
            result[side][key] = [value * factor for value in source[side][key]]
        result[side]["wheelCenterZ"] *= factor
    return result


def reflected_design(source):
    result = deepcopy(source)
    for side, other in (("left", "right"), ("right", "left")):
        result[side] = deepcopy(source[other])
        for key in ("axisUpper", "axisLower", "tieInner", "tieOuter"):
            x, y, z = source[other][key]
            result[side][key] = [x, -y, z]
    return result


def construction_design():
    def side(y, outer_y, inner_y):
        return {"axisUpper": [0, y, -10], "axisLower": [0, y, 10],
                "tieOuter": [-1, outer_y, 0], "tieInner": [-1, inner_y, 0]}
    return {"wheelbase": 10, "left": side(-2, -1, 1), "right": side(2, 1, -1)}
