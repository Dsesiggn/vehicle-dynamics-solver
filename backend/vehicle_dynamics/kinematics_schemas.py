"""Strict boundary for v2 axle-local designs and bounded rig sweeps."""
from typing import Annotated, Literal
from pydantic import BaseModel, ConfigDict, Field, model_validator

Finite = Annotated[float, Field(strict=True, allow_inf_nan=False)]
Coord = Annotated[Finite, Field(ge=-10000, le=10000)]
Point = Annotated[list[Coord], Field(min_length=3, max_length=3)]
OptionalAngle = Annotated[Finite, Field(ge=-45, le=45)] | None


class AlignmentInput(BaseModel):
    model_config = ConfigDict(extra="forbid")
    staticCamberDeg: OptionalAngle = None
    staticToeInDeg: OptionalAngle = None
    rockerAxis: Annotated[list[Coord | None], Field(min_length=3, max_length=3)] = [None, None, None]
    brakeReaction: Literal["upright","chassis"] | None = None
    driveReaction: Literal["upright","chassis"] | None = None
    arbModel: Literal["straight-transverse"] | None = None
    arbAttachment: Literal["upright","lower-wishbone","upper-wishbone","rocker"] | None = None

    @model_validator(mode="after")
    def axis_not_zero(self):
        if all(x is not None for x in self.rockerAxis) and sum(x*x for x in self.rockerAxis) < 1e-18:
            raise ValueError("Rocker axis must be nonzero, or unknown components left blank.")
        return self


class RigReference(BaseModel):
    model_config = ConfigDict(extra="forbid")
    x: Annotated[Finite, Field(ge=-20000, le=20000)] | None = None
    y: Annotated[Finite, Field(ge=-20000, le=20000)] | None = None


class AlignmentAxles(BaseModel):
    model_config = ConfigDict(extra="forbid")
    front: AlignmentInput = Field(default_factory=AlignmentInput)
    rear: AlignmentInput = Field(default_factory=AlignmentInput)


class ForceReference(BaseModel):
    model_config = ConfigDict(extra="forbid")
    cgHeightMm: Annotated[Finite, Field(gt=0, le=10000)] | None = None
    frontBrakeFraction: Annotated[Finite, Field(ge=0, le=1)] | None = None
    frontDriveFraction: Annotated[Finite, Field(ge=0, le=1)] | None = None


class KinematicsSettings(BaseModel):
    model_config = ConfigDict(extra="forbid")
    reference: RigReference = Field(default_factory=RigReference)
    contactModel: Literal["rigid-circle"] = "rigid-circle"
    axles: AlignmentAxles = Field(default_factory=AlignmentAxles)
    forceReference: ForceReference = Field(default_factory=ForceReference)


DraftPoint = Annotated[list[Coord | None], Field(min_length=3, max_length=3)]


class ArbPoints(BaseModel):
    model_config = ConfigDict(extra="forbid")
    bearing: DraftPoint = [None, None, None]
    bend: DraftPoint = [None, None, None]
    arm_tip: DraftPoint = [None, None, None]
    link_pickup: DraftPoint = [None, None, None]

    @model_validator(mode="after")
    def left_side(self):
        if any(p[1] is not None and p[1] > 0 for p in (self.bearing, self.bend, self.arm_tip, self.link_pickup)):
            raise ValueError("Left ARB points require nonpositive Y.")
        return self


class AntiRollBar(BaseModel):
    model_config = ConfigDict(extra="forbid")
    type: Literal["u-bar"] = "u-bar"
    enabled: Annotated[bool, Field(strict=True)] = False
    points: ArbPoints = Field(default_factory=ArbPoints)


class SuspensionAxle(BaseModel):
    # Retain packaging metadata; it does not supply missing constraints.
    model_config = ConfigDict(extra="allow")
    topology: Literal["double-wishbone", "macpherson", "multi-link"]
    track: Annotated[Finite, Field(ge=600, le=3000)]
    rackLength: Annotated[Finite, Field(ge=100, le=2500)]
    rackTravel: Annotated[Finite, Field(ge=1, le=300)]
    steered: Annotated[bool, Field(strict=True)]
    actuation: Literal["direct", "pushrod", "pullrod"]
    mounting: Literal["direct", "bell-crank"]
    actuationAttachment: Literal["upright", "lower-wishbone", "upper-wishbone"] = "upright"
    hardpoints: dict[str, Point]
    antiRollBar: AntiRollBar = Field(default_factory=AntiRollBar)

    @model_validator(mode="after")
    def geometry_contract(self):
        required = {"wheel_center", "tie_rod_inner", "tie_rod_outer"}
        if self.topology == "multi-link":
            required.update(f"{arm}_{end}" for arm in ("lower_front", "lower_rear", "upper_front", "upper_rear") for end in ("inner", "outer"))
        else:
            required.update(f"lca_{end}" for end in ("front", "rear", "outer"))
            required.update(("strut_top", "strut_bottom") if self.topology == "macpherson" else ("uca_front", "uca_rear", "uca_outer"))
        if self.topology != "macpherson":
            required.update(("actuation_outer", "damper_top"))
            if self.actuation != "direct":
                required.update(("rocker_pivot", "rocker_rod", "rocker_damper"))
        if set(self.hardpoints) != required:
            raise ValueError("Hardpoint keys must exactly match the selected suspension and actuator topology.")
        if self.mounting != ("direct" if self.actuation == "direct" else "bell-crank") or self.topology == "macpherson" and self.actuation != "direct":
            raise ValueError("Incompatible topology, actuation and mounting.")
        if self.actuationAttachment != "upright" and self.topology != "double-wishbone":
            raise ValueError("Wishbone attachment requires double-wishbone topology.")
        if abs(-2*self.hardpoints["wheel_center"][1]-self.track) > .1:
            raise ValueError("Left wheel center Y must equal negative half track.")
        if abs(-2*self.hardpoints["tie_rod_inner"][1]-self.rackLength) > .1 or self.rackLength >= self.track:
            raise ValueError("Rack width and left rack joint disagree or exceed track.")
        return self


class SuspensionAxles(BaseModel):
    model_config = ConfigDict(extra="forbid")
    front: SuspensionAxle
    rear: SuspensionAxle


class SuspensionDesign(BaseModel):
    model_config = ConfigDict(extra="allow")
    version: Literal[2]
    coordinateSystem: Literal["SAE_J670_Z_DOWN"]
    hardpointFrame: Literal["AXLE_LOCAL"]
    lengthUnit: Literal["mm"]
    name: Annotated[str, Field(strict=True, min_length=1, max_length=80)]
    wheelbase: Annotated[Finite, Field(ge=500, le=10000)]
    wheelDiameter: Annotated[Finite, Field(ge=200, le=1500)]
    axles: SuspensionAxles
    kinematics: KinematicsSettings = Field(default_factory=KinematicsSettings)

    @model_validator(mode="after")
    def nonblank_name(self):
        if not self.name.strip():
            raise ValueError("Design name cannot be blank.")
        return self


class RigSweep(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mode: Literal["heave", "pitch", "roll", "steering"]
    min: Finite
    max: Finite
    samples: Annotated[int, Field(strict=True, ge=3, le=81)] = 41

    @model_validator(mode="after")
    def limits(self):
        limit = 10 if self.mode in ("pitch", "roll") else 100 if self.mode == "heave" else 150
        if not -limit <= self.min < self.max <= limit:
            raise ValueError(f"Sweep endpoints must increase and remain within +/-{limit}.")
        return self


class KinematicsSweepRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    design: SuspensionDesign
    sweep: RigSweep

    @model_validator(mode="after")
    def reference_required(self):
        if self.sweep.mode == "pitch" and self.design.kinematics.reference.x is None:
            raise ValueError("Enter the front-origin rig pitch reference X before a pitch sweep.")
        if self.sweep.mode == "roll" and self.design.kinematics.reference.y is None:
            raise ValueError("Enter rig roll reference Y before a roll sweep.")
        if self.sweep.mode == "steering":
            for axle in (self.design.axles.front, self.design.axles.rear):
                if axle.steered and max(abs(self.sweep.min), abs(self.sweep.max)) > axle.rackTravel/2:
                    raise ValueError("Steering sweep exceeds a steered axle's declared rack stroke.")
        return self
