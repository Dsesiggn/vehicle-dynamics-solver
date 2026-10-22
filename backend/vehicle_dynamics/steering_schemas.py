"""HTTP contracts for the fixed-axis linkage; saved design v1 is unchanged."""
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .models.steering import validate_steering_design

Finite = Annotated[float, Field(strict=True, allow_inf_nan=False)]
Coordinate = Annotated[Finite, Field(ge=-1e7, le=1e7)]
Point = Annotated[list[Finite], Field(min_length=3, max_length=3)]
InputPoint = Annotated[list[Coordinate], Field(min_length=3, max_length=3)]
Dimension = Annotated[Finite, Field(gt=0, le=1e7)]
Alignment = Annotated[Finite, Field(gt=-90, lt=90)]
WheelInput = Annotated[Finite, Field(ge=-3600, le=3600)]


class SteeringSide(BaseModel):
    # Existing v1 drafts permit extensions; they do not affect this model.
    model_config = ConfigDict(extra="allow")
    axisUpper: InputPoint
    axisLower: InputPoint
    tieInner: InputPoint
    tieOuter: InputPoint
    wheelCenterZ: Coordinate
    staticToeDeg: Alignment
    staticCamberDeg: Alignment


class SteeringDesign(BaseModel):
    model_config = ConfigDict(extra="allow")
    version: Literal[1]
    kind: Literal["fixed-axis-steering"]
    coordinateSystem: Literal["SAE_J670_Z_DOWN"]
    hardpointFrame: Literal["AXLE_LOCAL"]
    lengthUnit: Literal["mm"]
    name: Annotated[str, Field(strict=True, min_length=1, max_length=200)]
    wheelbase: Dimension
    frontTrack: Dimension
    rearTrack: Dimension
    rackTravel: Dimension
    rackMmPerRevolution: Coordinate
    steeringLimitDeg: Annotated[Finite, Field(gt=0, le=3600, multiple_of=5)]
    left: SteeringSide
    right: SteeringSide

    @model_validator(mode="before")
    @classmethod
    def validate_without_coercion(cls, value):
        # This also rejects bool metadata/coordinates and blank names before
        # Pydantic's Literal/float handling can reinterpret them.
        errors = validate_steering_design(value)
        if errors:
            raise ValueError(" ".join(errors))
        return value


class SteeringSweepRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    design: SteeringDesign


class SteeringSolveRequest(SteeringSweepRequest):
    steeringWheelDeg: WheelInput


class SidePose(BaseModel):
    ok: Literal[True]
    headingDeg: Finite
    toeInDeg: Finite
    deltaToeInDeg: Finite
    camberDeg: Finite
    rotationDeg: Finite
    outer: Point
    inner: Point
    wheelCenter: Point
    spindle: Point
    tieLength: Finite
    residual: Finite


class SteeringMetrics(BaseModel):
    turn: Literal["straight", "mixed", "left", "right"]
    insideSide: Literal["left", "right"] | None
    insideDeg: Finite | None
    outsideDeg: Finite | None
    totalToeInDeg: Finite
    deltaTotalToeInDeg: Finite
    idealOutsideDeg: Finite | None
    ackermannErrorDeg: Finite | None
    ackermannPercent: Finite | None
    cotResidual: Finite | None


class ArmConstruction(BaseModel):
    axisFoot: Point | None
    planAxisPoint: Point | None
    outer: Point | None
    inner: Point | None
    spatialAngleDeg: Finite | None
    planAngleDeg: Finite | None


class ArmIntersection(BaseModel):
    status: Literal["finite", "parallel", "coincident", "undefined"]
    point: Point | None
    rearAxleOffsetMm: Finite | None


class SteeringConstruction(BaseModel):
    left: ArmConstruction
    right: ArmConstruction
    intersection: ArmIntersection
    pose: Literal["static", "solved"]


class SteeringSample(BaseModel):
    steeringWheelDeg: Finite
    rack: Finite | None
    construction: SteeringConstruction


class SolvedSteeringSample(SteeringSample):
    ok: Literal[True]
    left: SidePose
    right: SidePose
    metrics: SteeringMetrics


class FailedSteeringSample(SteeringSample):
    ok: Literal[False]
    reason: str
    left: None
    right: None
    metrics: None


SteeringResult = Annotated[SolvedSteeringSample | FailedSteeringSample, Field(discriminator="ok")]


class SteeringResponse(BaseModel):
    model: Literal["fixed_axis_steering_v1"] = "fixed_axis_steering_v1"
    coordinate_system: Literal["SAE_J670_Z_DOWN"] = "SAE_J670_Z_DOWN"
    hardpoint_frame: Literal["AXLE_LOCAL"] = "AXLE_LOCAL"
    length_unit: Literal["mm"] = "mm"
    angle_unit: Literal["deg"] = "deg"


class SteeringSolveResponse(SteeringResponse):
    result: SteeringResult


class SteeringSweepResponse(SteeringResponse):
    step_deg: Literal[5] = 5
    rows: Annotated[list[SteeringResult], Field(min_length=3, max_length=1441)]
